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
    output logic [ID_WIDTH-1:0]        m_axi_arid,
    output logic [ADDR_WIDTH-1:0]      m_axi_araddr,
    output logic [7:0]                 m_axi_arlen,
    output logic [2:0]                 m_axi_arsize,
    output logic [1:0]                 m_axi_arburst,
    output logic                       m_axi_arlock,
    output logic [3:0]                 m_axi_arcache,
    output logic [2:0]                 m_axi_arprot,
    output logic                       m_axi_arvalid,
    input  logic                       m_axi_arready,
    input  logic [ID_WIDTH-1:0]        m_axi_rid,
    input  logic [AXI_DATA_WIDTH-1:0]  m_axi_rdata,
    input  logic [1:0]                 m_axi_rresp,
    input  logic                       m_axi_rlast,
    input  logic                       m_axi_rvalid,
    output logic                       m_axi_rready,

    // ----------------------------------------------------------------------
    // AXI4 write master -- destination
    // ----------------------------------------------------------------------
    output logic [ID_WIDTH-1:0]        m_axi_awid,
    output logic [ADDR_WIDTH-1:0]      m_axi_awaddr,
    output logic [7:0]                 m_axi_awlen,
    output logic [2:0]                 m_axi_awsize,
    output logic [1:0]                 m_axi_awburst,
    output logic                       m_axi_awlock,
    output logic [3:0]                 m_axi_awcache,
    output logic [2:0]                 m_axi_awprot,
    output logic                       m_axi_awvalid,
    input  logic                       m_axi_awready,
    output logic [AXI_DATA_WIDTH-1:0]  m_axi_wdata,
    output logic [STRB_WIDTH-1:0]      m_axi_wstrb,
    output logic                       m_axi_wlast,
    output logic                       m_axi_wvalid,
    input  logic                       m_axi_wready,
    input  logic [ID_WIDTH-1:0]        m_axi_bid,
    input  logic [1:0]                 m_axi_bresp,
    input  logic                       m_axi_bvalid,
    output logic                       m_axi_bready
    );

   // soft reset (CSR-driven) folds into the datapath reset, not the CSRs
   logic       dma_soft_rst;
   wire        rst_dp = rst_sys | dma_soft_rst;

   // ----------------------------------------------------------------------
   // CSR slave <-> FSM
   // ----------------------------------------------------------------------
   dma_desc_t   desc;
   logic        desc_go;
   logic        dma_enable;
   logic        dma_ring_en;
   logic [31:0] ring_base;
   logic [ 4:0] ring_size;
   logic [31:0] ring_tail;
   logic [31:0] ring_head;
   logic        busy, idle;
   logic [ 7:0] fsm_state;
   logic [31:0] fence_seqno_r;
   logic        done_set;

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
      .dma_ring_en (dma_ring_en),
      .desc        (desc),
      .desc_go     (desc_go),
      .ring_base   (ring_base),
      .ring_size   (ring_size),
      .ring_tail   (ring_tail),
      .ring_head   (ring_head),
      .busy        (busy),
      .idle        (idle),
      .dma_error   (1'b0),         // no error path yet
      .state       (fsm_state),
      .fence_seqno (fence_seqno_r),
      .err_info    (32'h0),
      .done_set    (done_set),
      .err_set     (1'b0),
      .irq         (irq));

   // ----------------------------------------------------------------------
   // Command FSM state. Declared up front because the copy FIFO write is
   // gated off during a descriptor fetch -- the fetch reuses the source read
   // master to read one beat (= one descriptor) from the ring.
   // ----------------------------------------------------------------------
   typedef enum logic [3:0] {
      ST_IDLE       = 4'd0,
      ST_FETCH      = 4'd1,   // issue a one-beat descriptor read
      ST_FETCH_WAIT = 4'd2,   // capture the fetched descriptor
      ST_DISPATCH   = 4'd3,   // decode + set up the row walk
      ST_ROW_ISSUE  = 4'd4,   // copy/fill: launch rd+wr for one row
      ST_ROW_WAIT   = 4'd5,   // copy/fill: wait for the row to retire, then next row
      ST_BL_SRC     = 4'd6,   // blend: read the source row into the src FIFO
      ST_BL_SRC_W   = 4'd7,   // blend: wait for the source row read to retire
      ST_BL_DSTWR   = 4'd8,   // blend: read dst row into dst FIFO + blend-write it back
      ST_BL_DSTWR_W = 4'd9,   // blend: wait for the blend-write to retire, then next row
      ST_COMPLETE   = 4'd10   // advance fence (+ ring head)
   } dma_state_t;
   dma_state_t state;

   dma_desc_t                 cur_desc;     // command in flight (PIO or ring entry)
   logic                      from_ring;    // cur_desc came from the ring
   logic [AXI_DATA_WIDTH-1:0] fetch_beat;   // captured ring descriptor beat
   logic [31:0]               src_row_addr; // current row source byte address
   logic [31:0]               dst_row_addr; // current row dest byte address
   logic [31:0]               rows_left;    // rows still to copy (height down to 0)

   wire fetching = (state == ST_FETCH) | (state == ST_FETCH_WAIT);

   wire [7:0]  opcode     = cur_desc.opflags[7:0];
   wire        is_copy    = (opcode == DMA_OP_COPY2D);
   wire        is_fill    = (opcode == DMA_OP_FILL);
   wire        is_blend   = (opcode == DMA_OP_BLEND);

   // FILL pattern: the 32-bit ARGB color in src_addr, replicated across a beat
   localparam int PIX_PER_BEAT = AXI_DATA_WIDTH / 32;
   wire [AXI_DATA_WIDTH-1:0] fill_data = {PIX_PER_BEAT{cur_desc.src_addr}};

   // Sub-beat byte offsets of the current row's source/destination. A BLEND can
   // compose a source whose offset differs from the destination's: the source
   // read is realigned (u_realign) to the destination's offset, and the write
   // masks the partial first/last beats. COPY2D/FILL stay on the natural grid
   // (offsets forced 0) -- byte-identical to before this revision.
   wire                  realign_en  = is_blend;
   wire [BEAT_LSB-1:0]   src_off     = realign_en ? src_row_addr[BEAT_LSB-1:0] : '0;
   wire [BEAT_LSB-1:0]   dst_off     = realign_en ? dst_row_addr[BEAT_LSB-1:0] : '0;

   // Per-pass beats: ceil((off + width)/beat). The source span (read) and the
   // destination span (read-back + write) can differ by one beat; the realigner
   // consumes src_row_beats and emits dst_row_beats.
   wire [31:0] src_row_beats = (32'(src_off) + cur_desc.width + 32'(STRB_WIDTH-1)) >> BEAT_LSB;
   wire [31:0] dst_row_beats = (32'(dst_off) + cur_desc.width + 32'(STRB_WIDTH-1)) >> BEAT_LSB;

   // Write byte-strobes: head masks the low dst_off bytes of the first beat,
   // tail masks the partial last beat ((dst_off + width) mod beat). With
   // dst_off = 0 the head is all-ones and the tail reduces to the old width-tail.
   wire [STRB_WIDTH-1:0] wr_head_be = {STRB_WIDTH{1'b1}} << dst_off;
   wire [BEAT_LSB:0]     tail_sum   = {1'b0, dst_off} + {1'b0, cur_desc.width[BEAT_LSB-1:0]};
   wire [BEAT_LSB-1:0]   tail_bytes = tail_sum[BEAT_LSB-1:0];
   wire [STRB_WIDTH-1:0] wr_tail_be = (tail_bytes == '0)
                                    ? {STRB_WIDTH{1'b1}}
                                    : (STRB_WIDTH'(1) << tail_bytes) - STRB_WIDTH'(1);

   // ring fetch address: ring_base + (head mod entries) * 32 bytes
   wire [31:0] ring_mask = (32'd1 << ring_size) - 32'd1;
   wire [31:0] ring_addr = ring_base + ((ring_head & ring_mask) << BEAT_LSB);

   // ----------------------------------------------------------------------
   // Read / write master controls. rd serves the descriptor fetch (one beat),
   // the copy/blend source row, and the blend destination read-back; the FSM
   // never overlaps these uses of the single read port.
   //
   //   blend_dst_pass : the read in flight is the blend destination read-back,
   //                    routed to the dst FIFO (otherwise reads fill the src
   //                    FIFO). Held across the read so its tail beat lands in
   //                    the dst FIFO before the state advances.
   // ----------------------------------------------------------------------
   logic rd_busy, rd_done;
   logic wr_busy, wr_done;

   wire blend_dst_pass = (state == ST_BL_DSTWR) | (state == ST_BL_DSTWR_W);

   wire rd_start = (state == ST_FETCH)                  |  // ring descriptor fetch
                   ((state == ST_ROW_ISSUE) & is_copy)  |  // copy source row (FILL reads nothing)
                   (state == ST_BL_SRC)                 |  // blend source row   -> src FIFO
                   (state == ST_BL_DSTWR);                 // blend dest readback -> dst FIFO
   wire wr_start = (state == ST_ROW_ISSUE) | (state == ST_BL_DSTWR);

   wire [ADDR_WIDTH-1:0] rd_base  = fetching       ? ring_addr[ADDR_WIDTH-1:0]    :
                                    blend_dst_pass ? dst_row_addr[ADDR_WIDTH-1:0] :
                                                     src_row_addr[ADDR_WIDTH-1:0];
   wire [31:0]          rd_nbeats = fetching       ? 32'd1 :
                                    blend_dst_pass ? dst_row_beats :
                                                     src_row_beats;
   wire [ADDR_WIDTH-1:0] wr_base  = dst_row_addr[ADDR_WIDTH-1:0];

   // ----------------------------------------------------------------------
   // Two copy FIFOs. The src FIFO holds copy data / the blend source row; the
   // dst FIFO holds the blend destination read-back. The read master fills
   // whichever the current pass selects (blend_dst_pass), throttling on that
   // FIFO's occupancy. The write master drains the src FIFO for a copy, or the
   // per-beat SRC_OVER of (src,dst) FIFO heads for a blend.
   // ----------------------------------------------------------------------
   // Raw source beat accepted into the source path (copy/blend source row).
   wire                       src_beat = m_axi_rvalid & m_axi_rready & ~fetching & ~blend_dst_pass;
   // BLEND routes the source through the realigner (re-aligns to the dst offset);
   // COPY2D/FILL keep the direct, beat-aligned path untouched.
   wire                       realign_out_valid;
   wire [AXI_DATA_WIDTH-1:0]  realign_out_data;
   wire                       realign_busy;
   wire                       fifo_s_wen = is_blend ? realign_out_valid : src_beat;
   wire [AXI_DATA_WIDTH-1:0]  fifo_s_din = is_blend ? realign_out_data  : m_axi_rdata;
   wire [FIFO_LGDEPTH:0]      fifo_s_fill;
   wire                       fifo_s_ren;
   wire [AXI_DATA_WIDTH-1:0]  fifo_s_dout;
   wire                       fifo_s_empty;

   wire                       fifo_d_wen = m_axi_rvalid & m_axi_rready & ~fetching & blend_dst_pass;
   wire [FIFO_LGDEPTH:0]      fifo_d_fill;
   wire                       fifo_d_ren;
   wire [AXI_DATA_WIDTH-1:0]  fifo_d_dout;
   wire                       fifo_d_empty;

   sfifo #
     (.WIDTH (AXI_DATA_WIDTH),
      .LGSIZ (FIFO_LGDEPTH))
   u_fifo_s
     (.clk   (clk_sys),
      .rst   (rst_dp),
      .wen   (fifo_s_wen),
      .din   (fifo_s_din),
      .full  (),
      .fill  (fifo_s_fill),
      .ren   (fifo_s_ren),
      .dout  (fifo_s_dout),
      .empty (fifo_s_empty));

   // ----------------------------------------------------------------------
   // Source realigner (BLEND only). Re-cuts the beat-aligned source read
   // stream into one aligned to the destination's sub-beat offset, so a sprite
   // can be composited at an arbitrary destination X. Started at the blend
   // source-row launch; copy/fill bypass it (is_blend == 0).
   // ----------------------------------------------------------------------
   vctrl_dma_realign #
     (.AXI_DATA_WIDTH (AXI_DATA_WIDTH))
   u_realign
     (.clk       (clk_sys),
      .rst       (rst_dp),
      .start     (state == ST_BL_SRC),
      .src_off   (src_off),
      .dst_off   (dst_off),
      .out_beats (dst_row_beats),
      .in_beats  (src_row_beats),
      .in_valid  (src_beat & is_blend),
      .in_data   (m_axi_rdata),
      .out_valid (realign_out_valid),
      .out_data  (realign_out_data),
      .busy      (realign_busy));

   sfifo #
     (.WIDTH (AXI_DATA_WIDTH),
      .LGSIZ (FIFO_LGDEPTH))
   u_fifo_d
     (.clk   (clk_sys),
      .rst   (rst_dp),
      .wen   (fifo_d_wen),
      .din   (m_axi_rdata),
      .full  (),
      .fill  (fifo_d_fill),
      .ren   (fifo_d_ren),
      .dout  (fifo_d_dout),
      .empty (fifo_d_empty));

   // throttle the read master on the FIFO it is currently filling
   wire [FIFO_LGDEPTH:0] rd_fifo_fill = blend_dst_pass ? fifo_d_fill : fifo_s_fill;

   // ----------------------------------------------------------------------
   // Blend pipeline. The SRC_OVER ALU has a multi-cycle (pipelined-multiply)
   // datapath, so it cannot sit combinationally between the FIFOs and writer. A
   // feeder pops one src+dst pair per cycle into the pipeline whenever both are
   // available and the blended-output FIFO has room for every beat that would
   // then be in flight; the writer drains that output FIFO. The margin (>=
   // pipeline latency) bounds overflow once the writer back-pressures. Copy and
   // fill never enter here -- the writer drains the src FIFO (or its own
   // pattern) directly.
   // ----------------------------------------------------------------------
   localparam int BLEND_LAT      = 5;                 // == vctrl_dma_blend LATENCY
   localparam int FIFO_O_LGDEPTH = 4;
   localparam int FIFO_O_DEPTH   = (1 << FIFO_O_LGDEPTH);
   // stop feeding with room for every in-flight beat (>= pipeline latency)
   localparam logic [FIFO_O_LGDEPTH:0] FIFO_O_HIWAT =
              (FIFO_O_LGDEPTH+1)'(FIFO_O_DEPTH - 2*BLEND_LAT);

   wire                       wr_fifo_rd;
   wire                       fifo_o_ren = is_blend & wr_fifo_rd;
   wire [FIFO_O_LGDEPTH:0]    fifo_o_fill;
   wire [AXI_DATA_WIDTH-1:0]  fifo_o_dout;
   wire                       fifo_o_empty;

   wire blend_inject = is_blend & ~fifo_s_empty & ~fifo_d_empty &
                       (fifo_o_fill < FIFO_O_HIWAT);

   wire                       blend_out_valid;
   wire [AXI_DATA_WIDTH-1:0]  blend_out;

   vctrl_dma_blend #
     (.AXI_DATA_WIDTH (AXI_DATA_WIDTH))
   u_blend
     (.clk       (clk_sys),
      .rst       (rst_dp),
      .in_valid  (blend_inject),
      .s         (fifo_s_dout),
      .d         (fifo_d_dout),
      .out_valid (blend_out_valid),
      .o         (blend_out));

   sfifo #
     (.WIDTH (AXI_DATA_WIDTH),
      .LGSIZ (FIFO_O_LGDEPTH))
   u_fifo_o
     (.clk   (clk_sys),
      .rst   (rst_dp),
      .wen   (blend_out_valid),
      .din   (blend_out),
      .full  (),
      .fill  (fifo_o_fill),
      .ren   (fifo_o_ren),
      .dout  (fifo_o_dout),
      .empty (fifo_o_empty));

   // write-data source: copy drains the src FIFO; blend drains the blended
   // output FIFO; fill drives its own pattern in u_wr
   wire [AXI_DATA_WIDTH-1:0] wr_din   = is_blend ? fifo_o_dout  : fifo_s_dout;
   wire                      wr_empty = is_blend ? fifo_o_empty : fifo_s_empty;

   assign fifo_s_ren = is_blend ? blend_inject : wr_fifo_rd;  // blend pops src via the feeder
   assign fifo_d_ren = is_blend ? blend_inject : 1'b0;        // blend pops dst via the feeder

   assign busy      = (state != ST_IDLE);
   assign idle      = (state == ST_IDLE) & ~rd_busy & ~wr_busy;
   assign fsm_state = {4'h0, state};

   // A blend buffers the whole source row in the src FIFO before the dst pass
   // drains it, so a row must fit (minus the read master's burst reserve).
   // synthesis translate_off
   localparam int BL_MAX_ROW_BEATS = (1 << FIFO_LGDEPTH) - 2*BURST_LEN;
   always_ff @(posedge clk_sys)
     if (~rst_dp & (state == ST_DISPATCH) & is_blend)
       // +1 covers the extra beat a non-zero sub-beat offset can add
       assert (((cur_desc.width + 32'(STRB_WIDTH-1)) >> BEAT_LSB) + 32'd1 <= BL_MAX_ROW_BEATS)
         else $error("BLEND row of %0d bytes exceeds src-FIFO capacity %0d beats",
                     cur_desc.width, BL_MAX_ROW_BEATS);
   // synthesis translate_on

   always_ff @(posedge clk_sys) begin
      done_set <= 1'b0;

      unique case (state)
        ST_IDLE: begin
           if (desc_go & dma_enable) begin
              cur_desc  <= desc;        // PIO submit
              from_ring <= 1'b0;
              state     <= ST_DISPATCH;
           end
           else if (dma_enable & dma_ring_en & (ring_tail != ring_head)) begin
              state <= ST_FETCH;        // the ring has work
           end
        end

        ST_FETCH: state <= ST_FETCH_WAIT;   // rd_start issued this cycle

        ST_FETCH_WAIT: begin
           if (m_axi_rvalid & m_axi_rready)
             fetch_beat <= m_axi_rdata;
           if (rd_done) begin
              cur_desc  <= dma_desc_t'(fetch_beat);
              from_ring <= 1'b1;
              state     <= ST_DISPATCH;
           end
        end

        ST_DISPATCH: begin
           if (is_copy | is_fill | is_blend) begin
              src_row_addr <= cur_desc.src_addr;
              dst_row_addr <= cur_desc.dst_addr;
              rows_left    <= cur_desc.height;
              state        <= is_blend ? ST_BL_SRC : ST_ROW_ISSUE;
           end
           else
             state <= ST_COMPLETE;       // NOP / FENCE
        end

        ST_ROW_ISSUE: state <= ST_ROW_WAIT;   // rd+wr launched this cycle

        ST_ROW_WAIT:
          if (wr_done) begin
             if (rows_left > 32'd1) begin
                src_row_addr <= src_row_addr + cur_desc.src_pitch;
                dst_row_addr <= dst_row_addr + cur_desc.dst_pitch;
                rows_left    <= rows_left - 1'b1;
                state        <= ST_ROW_ISSUE;   // next row
             end
             else
               state <= ST_COMPLETE;            // last row done
          end

        // ---- blend: read source row -> src FIFO ----
        ST_BL_SRC:   state <= ST_BL_SRC_W;      // rd (src) launched this cycle
        // wait for the source read to retire (rd idle) AND the realigner to
        // flush its last beat into the src FIFO before draining it in the dst
        // pass -- both are level signals (rd_done is only a 1-cycle pulse and
        // the realigner's tail flush can trail it).
        ST_BL_SRC_W: if (~rd_busy & ~realign_busy) state <= ST_BL_DSTWR;

        // ---- blend: read dst row -> dst FIFO, compositing-write it back ----
        // rd (dst) and wr (blend) launch together and stream through the FIFOs.
        // The writer only emits a beat once it has been popped from the dst
        // FIFO -- i.e. after that beat's dst read has returned -- so the write
        // of dst[i] always follows the read of dst[i]: no read/write hazard.
        ST_BL_DSTWR:   state <= ST_BL_DSTWR_W;  // rd+wr launched this cycle
        ST_BL_DSTWR_W:
          if (wr_done) begin
             if (rows_left > 32'd1) begin
                src_row_addr <= src_row_addr + cur_desc.src_pitch;
                dst_row_addr <= dst_row_addr + cur_desc.dst_pitch;
                rows_left    <= rows_left - 1'b1;
                state        <= ST_BL_SRC;      // next row
             end
             else
               state <= ST_COMPLETE;            // last row done
          end

        ST_COMPLETE: begin
           // write retiring last (B received) is the true completion
           fence_seqno_r <= cur_desc.seqno;
           done_set      <= 1'b1;
           if (from_ring) ring_head <= ring_head + 1'b1;
           state         <= ST_IDLE;
        end

        default: state <= ST_IDLE;
      endcase

      if (rst_dp) begin
         state         <= ST_IDLE;
         fence_seqno_r <= '0;
         done_set      <= 1'b0;
         ring_head     <= '0;
         from_ring     <= 1'b0;
         src_row_addr  <= '0;
         dst_row_addr  <= '0;
         rows_left     <= '0;
      end
   end

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
      .base          (rd_base),
      .nbeats        (rd_nbeats),
      .busy          (rd_busy),
      .done          (rd_done),
      .fifo_fill     (rd_fifo_fill),
      .m_axi_arid,
      .m_axi_araddr,
      .m_axi_arlen,
      .m_axi_arsize,
      .m_axi_arburst,
      .m_axi_arlock,
      .m_axi_arcache,
      .m_axi_arprot,
      .m_axi_arvalid,
      .m_axi_arready,
      .m_axi_rvalid,
      .m_axi_rlast,
      .m_axi_rready
      );

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
      .base          (wr_base),
      .nbeats        (dst_row_beats),
      .head_be       (wr_head_be),
      .last_be       (wr_tail_be),
      .busy          (wr_busy),
      .done          (wr_done),
      .fifo_rd       (wr_fifo_rd),
      .fifo_dout     (wr_din),
      .fifo_empty    (wr_empty),
      .fill_mode     (is_fill),
      .fill_data     (fill_data),
      .m_axi_awid,
      .m_axi_awaddr,
      .m_axi_awlen,
      .m_axi_awsize,
      .m_axi_awburst,
      .m_axi_awlock,
      .m_axi_awcache,
      .m_axi_awprot,
      .m_axi_awvalid,
      .m_axi_awready,
      .m_axi_wdata,
      .m_axi_wstrb,
      .m_axi_wlast,
      .m_axi_wvalid,
      .m_axi_wready,
      .m_axi_bid,
      .m_axi_bresp,
      .m_axi_bvalid,
      .m_axi_bready
      );

endmodule // vctrl_dma
