// -*- mode: verilog; mode: font-lock; indent-tabs-mode: nil -*-
// vi: set et ts=3 sw=3 sts=3:
//
// Copyright 2026 Steffen Persvold
// SPDX-License-Identifier: Apache-2.0
//
// ========================================================================
// File        : vctrl_dma.sv
// Author      : Steffen Persvold
// Created     : June 17, 2026
// ========================================================================
// Description : DMA engine top.
//
//   Ties the CSR slave (vctrl_dma_regs), the copy FIFO, and the source
//   read / destination write masters together with a small command FSM.
//   A doorbell write hands the FSM a descriptor; for a COPY2D it kicks both
//   masters (source -> FIFO -> destination) and, once the write has fully
//   retired (all B responses in), advances the fence seqno and pulses the
//   completion interrupt. The read and write AXI masters are independent
//   ports so the integration can route the source and destination to
//   different memories.
//
//   Phase 1a scope: contiguous (linear) copies -- DMA_WIDTH bytes, height 1,
//   beat-aligned. 2D stride (per-row walk) and partial-beat masking come
//   later; the descriptor already carries the fields.
// ========================================================================
//

module vctrl_dma import vctrl_pkg::*;
  #(
    parameter  integer ADDR_WIDTH     = 32,
    parameter  integer AXI_DATA_WIDTH = 256,
    parameter  integer ID_WIDTH       = 1,
    parameter  integer BURST_LEN      = 16,
    parameter  integer FIFO_LGDEPTH   = 9,
    localparam integer STRB_WIDTH     = AXI_DATA_WIDTH/8,
    localparam integer BEAT_LSB       = $clog2(STRB_WIDTH)
    )
   (
    input  logic                       clk_sys,
    input  logic                       rst_sys,

    // ----------------------------------------------------------------------
    // Command register bus slave (cmd_*)
    // ----------------------------------------------------------------------
    input  logic                       cmd_req,
    input  logic [11: 2]               cmd_adr,
    input  logic                       cmd_we,
    input  logic [ 3: 0]               cmd_be,
    input  logic [31: 0]               cmd_d,
    output logic [31: 0]               cmd_q,
    output logic                       cmd_ack,

    // ----------------------------------------------------------------------
    // Completion interrupt
    // ----------------------------------------------------------------------
    output logic                       irq,

    // ----------------------------------------------------------------------
    // AXI4 read master -- source
    // ----------------------------------------------------------------------
    output logic [ID_WIDTH-1:0]        m_axi_rd_arid,
    output logic [ADDR_WIDTH-1:0]      m_axi_rd_araddr,
    output logic [7:0]                 m_axi_rd_arlen,
    output logic [2:0]                 m_axi_rd_arsize,
    output logic [1:0]                 m_axi_rd_arburst,
    output logic                       m_axi_rd_arlock,
    output logic [3:0]                 m_axi_rd_arcache,
    output logic [2:0]                 m_axi_rd_arprot,
    output logic                       m_axi_rd_arvalid,
    input  logic                       m_axi_rd_arready,
    input  logic [ID_WIDTH-1:0]        m_axi_rd_rid,
    input  logic [AXI_DATA_WIDTH-1:0]  m_axi_rd_rdata,
    input  logic [1:0]                 m_axi_rd_rresp,
    input  logic                       m_axi_rd_rlast,
    input  logic                       m_axi_rd_rvalid,
    output logic                       m_axi_rd_rready,

    // ----------------------------------------------------------------------
    // AXI4 write master -- destination
    // ----------------------------------------------------------------------
    output logic [ID_WIDTH-1:0]        m_axi_wr_awid,
    output logic [ADDR_WIDTH-1:0]      m_axi_wr_awaddr,
    output logic [7:0]                 m_axi_wr_awlen,
    output logic [2:0]                 m_axi_wr_awsize,
    output logic [1:0]                 m_axi_wr_awburst,
    output logic                       m_axi_wr_awlock,
    output logic [3:0]                 m_axi_wr_awcache,
    output logic [2:0]                 m_axi_wr_awprot,
    output logic                       m_axi_wr_awvalid,
    input  logic                       m_axi_wr_awready,
    output logic [AXI_DATA_WIDTH-1:0]  m_axi_wr_wdata,
    output logic [STRB_WIDTH-1:0]      m_axi_wr_wstrb,
    output logic                       m_axi_wr_wlast,
    output logic                       m_axi_wr_wvalid,
    input  logic                       m_axi_wr_wready,
    input  logic [ID_WIDTH-1:0]        m_axi_wr_bid,
    input  logic [1:0]                 m_axi_wr_bresp,
    input  logic                       m_axi_wr_bvalid,
    output logic                       m_axi_wr_bready
    );

   // soft reset (CSR-driven) folds into the datapath reset, not the CSRs
   logic       dma_soft_rst;
   wire        rst_dp = rst_sys | dma_soft_rst;

   // ----------------------------------------------------------------------
   // CSR slave <-> FSM
   // ----------------------------------------------------------------------
   dma_desc_t  desc;
   logic       desc_go;
   logic       dma_enable;
   logic       busy, idle;
   logic [7:0] fsm_state;
   logic [31:0] fence_seqno_r;
   logic       done_set;

   vctrl_dma_regs u_regs
     (.clk_sys     (clk_sys),
      .rst_sys     (rst_sys),
      .cmd_req     (cmd_req),
      .cmd_adr     (cmd_adr),
      .cmd_we      (cmd_we),
      .cmd_be      (cmd_be),
      .cmd_d       (cmd_d),
      .cmd_q       (cmd_q),
      .cmd_ack     (cmd_ack),
      .dma_enable  (dma_enable),
      .dma_soft_rst(dma_soft_rst),
      .desc        (desc),
      .desc_go     (desc_go),
      .busy        (busy),
      .idle        (idle),
      .dma_error   (1'b0),         // no error path in Phase 1a
      .state       (fsm_state),
      .fence_seqno (fence_seqno_r),
      .err_info    (32'h0),
      .done_set    (done_set),
      .err_set     (1'b0),
      .irq         (irq));

   // ----------------------------------------------------------------------
   // Copy FIFO : source read master fills, destination write master drains
   // ----------------------------------------------------------------------
   wire                       fifo_wen = m_axi_rd_rvalid & m_axi_rd_rready;
   wire [FIFO_LGDEPTH:0]      fifo_fill;
   wire                       fifo_ren;
   wire [AXI_DATA_WIDTH-1:0]  fifo_dout;
   wire                       fifo_empty;

   sfifo #
     (.WIDTH (AXI_DATA_WIDTH),
      .LGSIZ (FIFO_LGDEPTH))
   u_fifo
     (.clk   (clk_sys),
      .rst   (rst_dp),
      .wen   (fifo_wen),
      .din   (m_axi_rd_rdata),
      .full  (),
      .fill  (fifo_fill),
      .ren   (fifo_ren),
      .dout  (fifo_dout),
      .empty (fifo_empty));

   // ----------------------------------------------------------------------
   // Command FSM
   // ----------------------------------------------------------------------
   typedef enum logic [1:0] { ST_IDLE = 2'd0, ST_RUN = 2'd1 } dma_state_t;
   dma_state_t state;

   wire [7:0]   opcode     = desc.opflags[7:0];
   wire         is_copy    = (opcode == DMA_OP_COPY2D);
   wire [31:0]  job_nbeats = desc.width >> BEAT_LSB;     // linear: width bytes, height 1

   logic [31:0] job_seqno_r;
   logic        rd_start, rd_busy, rd_done;
   logic        wr_start, wr_busy, wr_done;

   // launch both engines off the doorbell for a copy
   assign rd_start = (state == ST_IDLE) & desc_go & dma_enable & is_copy;
   assign wr_start = rd_start;

   assign busy      = (state == ST_RUN);
   assign idle      = (state == ST_IDLE) & ~rd_busy & ~wr_busy;
   assign fsm_state = {6'h0, state};

   always_ff @(posedge clk_sys) begin
      done_set <= 1'b0;

      unique case (state)
        ST_IDLE: begin
           if (desc_go & dma_enable) begin
              if (is_copy)
                state <= ST_RUN;
              else begin
                 // NOP / FENCE / (unimplemented): complete immediately
                 fence_seqno_r <= desc.seqno;
                 done_set      <= 1'b1;
              end
           end
        end
        ST_RUN: begin
           // write retiring last (B received) is the true completion
           if (wr_done) begin
              fence_seqno_r <= job_seqno_r;
              done_set      <= 1'b1;
              state         <= ST_IDLE;
           end
        end
        default: state <= ST_IDLE;
      endcase

      if (rst_dp) begin
         state         <= ST_IDLE;
         fence_seqno_r <= '0;
         done_set      <= 1'b0;
      end
   end

   // latch the in-flight seqno so the right fence is reported even if the
   // descriptor registers are rewritten while the copy runs
   always_ff @(posedge clk_sys)
     if (rst_dp)
       job_seqno_r <= '0;
     else if (rd_start)
       job_seqno_r <= desc.seqno;

   // ----------------------------------------------------------------------
   // Source read master
   // ----------------------------------------------------------------------
   vctrl_dma_rd #
     (.ADDR_WIDTH     (ADDR_WIDTH),
      .AXI_DATA_WIDTH (AXI_DATA_WIDTH),
      .ID_WIDTH       (ID_WIDTH),
      .BURST_LEN      (BURST_LEN),
      .FIFO_LGDEPTH   (FIFO_LGDEPTH))
   u_rd
     (.clk           (clk_sys),
      .rst           (rst_dp),
      .start         (rd_start),
      .base          (desc.src_addr[ADDR_WIDTH-1:0]),
      .nbeats        (job_nbeats),
      .busy          (rd_busy),
      .done          (rd_done),
      .fifo_fill     (fifo_fill),
      .m_axi_arid    (m_axi_rd_arid),
      .m_axi_araddr  (m_axi_rd_araddr),
      .m_axi_arlen   (m_axi_rd_arlen),
      .m_axi_arsize  (m_axi_rd_arsize),
      .m_axi_arburst (m_axi_rd_arburst),
      .m_axi_arlock  (m_axi_rd_arlock),
      .m_axi_arcache (m_axi_rd_arcache),
      .m_axi_arprot  (m_axi_rd_arprot),
      .m_axi_arvalid (m_axi_rd_arvalid),
      .m_axi_arready (m_axi_rd_arready),
      .m_axi_rvalid  (m_axi_rd_rvalid),
      .m_axi_rlast   (m_axi_rd_rlast),
      .m_axi_rready  (m_axi_rd_rready));

   // ----------------------------------------------------------------------
   // Destination write master
   // ----------------------------------------------------------------------
   vctrl_dma_wr #
     (.ADDR_WIDTH     (ADDR_WIDTH),
      .AXI_DATA_WIDTH (AXI_DATA_WIDTH),
      .ID_WIDTH       (ID_WIDTH),
      .BURST_LEN      (BURST_LEN))
   u_wr
     (.clk           (clk_sys),
      .rst           (rst_dp),
      .start         (wr_start),
      .base          (desc.dst_addr[ADDR_WIDTH-1:0]),
      .nbeats        (job_nbeats),
      .busy          (wr_busy),
      .done          (wr_done),
      .fifo_rd       (fifo_ren),
      .fifo_dout     (fifo_dout),
      .fifo_empty    (fifo_empty),
      .m_axi_awid    (m_axi_wr_awid),
      .m_axi_awaddr  (m_axi_wr_awaddr),
      .m_axi_awlen   (m_axi_wr_awlen),
      .m_axi_awsize  (m_axi_wr_awsize),
      .m_axi_awburst (m_axi_wr_awburst),
      .m_axi_awlock  (m_axi_wr_awlock),
      .m_axi_awcache (m_axi_wr_awcache),
      .m_axi_awprot  (m_axi_wr_awprot),
      .m_axi_awvalid (m_axi_wr_awvalid),
      .m_axi_awready (m_axi_wr_awready),
      .m_axi_wdata   (m_axi_wr_wdata),
      .m_axi_wstrb   (m_axi_wr_wstrb),
      .m_axi_wlast   (m_axi_wr_wlast),
      .m_axi_wvalid  (m_axi_wr_wvalid),
      .m_axi_wready  (m_axi_wr_wready),
      .m_axi_bid     (m_axi_wr_bid),
      .m_axi_bresp   (m_axi_wr_bresp),
      .m_axi_bvalid  (m_axi_wr_bvalid),
      .m_axi_bready  (m_axi_wr_bready));

endmodule // vctrl_dma
