// -*- mode: verilog; mode: font-lock; indent-tabs-mode: nil -*-
// vi: set et ts=3 sw=3 sts=3:
//
// Copyright 2026 Steffen Persvold
// SPDX-License-Identifier: Apache-2.0
//
// ========================================================================
// File        : vctrl_dma_wr.sv
// Author      : Steffen Persvold
// Created     : June 17, 2026
// ========================================================================
// Description : DMA destination write master.
//
//   Drains the copy FIFO (held in the parent) and writes a contiguous run
//   of `nbeats` full-width beats to `base` as long INCR bursts (up to
//   BURST_LEN; short final burst). Address (AW) is issued ahead of data,
//   many-outstanding, to hide the destination round-trip; each accepted
//   burst's length is queued in a small length FIFO so the W engine emits
//   exactly that many beats with wlast in order, and B responses are
//   counted for completion.
//
//   Same hardened discipline as the read master: awvalid is registered and
//   held until awready, the pointer advances at launch, and AW launches are
//   throttled by length-FIFO room + an outstanding-burst cap. The final beat
//   of a row is masked by `last_be` for non-beat-aligned widths.
//
//   Fill mode (no source read): when `fill_mode` is asserted the W engine
//   drives the constant `fill_data` beat instead of the FIFO and does not pop
//   it, so a solid fill moves no read traffic.
// ========================================================================
//

module vctrl_dma_wr
  #(
    parameter  integer ADDR_WIDTH      = 32,
    parameter  integer AXI_DATA_WIDTH  = 256,
    parameter  integer ID_WIDTH        = 1,
    parameter  integer BURST_LEN       = 16,   // max beats per AW
    parameter  integer MAX_OUTSTANDING = 8,    // AW bursts in flight (AW..B)
    localparam integer STRB_WIDTH      = AXI_DATA_WIDTH/8,
    localparam integer AXSIZE          = $clog2(STRB_WIDTH),
    localparam integer BEAT_LSB        = $clog2(STRB_WIDTH),
    localparam integer LEN_LGDEPTH     = 4     // length FIFO = 16 bursts
    )
   (
    input  logic                       clk,
    input  logic                       rst,

    // ----------------------------------------------------------------------
    // Job control
    // ----------------------------------------------------------------------
    input  logic                       start,   // one-cycle: begin a transfer
    input  logic [ADDR_WIDTH-1:0]      base,    // dest byte address (beat aligned)
    input  logic [31:0]                nbeats,  // number of beats to write
    input  logic [STRB_WIDTH-1:0]      last_be, // write strobe for the final beat (partial tail)
    output logic                       busy,    // transfer in progress
    output logic                       done,    // one-cycle: all beats written + B'd

    // ----------------------------------------------------------------------
    // Copy FIFO read side (FIFO lives in the parent, show-ahead)
    // ----------------------------------------------------------------------
    output logic                       fifo_rd,
    input  logic [AXI_DATA_WIDTH-1:0]  fifo_dout,
    input  logic                       fifo_empty,

    // ----------------------------------------------------------------------
    // Fill mode: write a constant pattern beat, no FIFO/source read
    // ----------------------------------------------------------------------
    input  logic                       fill_mode,
    input  logic [AXI_DATA_WIDTH-1:0]  fill_data,

    // ----------------------------------------------------------------------
    // AXI4 write master
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

   localparam logic [1:0] BURST_INCR = 2'b01;

   logic                  running;

   // ----------------------------------------------------------------------
   // AW (address) engine
   // ----------------------------------------------------------------------
   logic [ADDR_WIDTH-1:0] aw_addr;
   logic [31:0]           aw_beats_left;
   logic [7:0]            bursts_out;    // AW accepted, B not yet received

   wire aw_acc     = m_axi_awvalid & m_axi_awready;
   wire aw_pending = m_axi_awvalid & ~m_axi_awready;
   wire [8:0] this_aw = (aw_beats_left >= BURST_LEN) ? 9'(BURST_LEN) : aw_beats_left[8:0];

   // Length FIFO: each accepted AW's beat count, consumed by the W engine.
   wire        len_full, len_empty;
   wire [8:0]  len_dout;
   logic       len_wen, len_ren;

   sfifo #
     (.WIDTH (9),
      .LGSIZ (LEN_LGDEPTH))
   u_lenfifo
     (.clk,
      .rst,
      .wen   (len_wen),
      .din   (9'(m_axi_awlen) + 1'b1),
      .full  (len_full),
      .fill  (),
      .ren   (len_ren),
      .dout  (len_dout),
      .empty (len_empty));

   wire b_acc = m_axi_bvalid & m_axi_bready;

   wire can_launch_aw = running & (aw_beats_left != '0) & ~aw_pending &
                        ~len_full & (bursts_out < 8'(MAX_OUTSTANDING));

   assign len_wen = aw_acc;     // queue the accepted burst length for the W engine

   assign m_axi_awid    = '0;
   assign m_axi_awsize  = 3'(AXSIZE);
   assign m_axi_awburst = BURST_INCR;
   assign m_axi_awlock  = 1'b0;
   assign m_axi_awcache = 4'b0011;
   assign m_axi_awprot  = 3'b000;
   assign m_axi_bready  = 1'b1;

   // ----------------------------------------------------------------------
   // W (data) engine : drains the copy FIFO, one queued burst at a time
   // ----------------------------------------------------------------------
   logic       w_active;
   logic [8:0] w_cnt;          // beats left in the current W burst
   logic [31:0] w_beats_left;  // beats left overall

   wire w_acc   = m_axi_wvalid & m_axi_wready;
   wire w_start = ~w_active & ~len_empty;   // begin the next queued burst

   assign len_ren      = w_start;
   assign m_axi_wdata  = fill_mode ? fill_data : fifo_dout;
   assign m_axi_wstrb  = (w_beats_left == 32'd1) ? last_be : '1;
   assign m_axi_wvalid = w_active & (fill_mode | ~fifo_empty);
   assign m_axi_wlast  = w_active & (w_cnt == 9'd1);
   assign fifo_rd      = w_acc & ~fill_mode;

   assign busy = running;

   wire all_done = running & (aw_beats_left == '0) & (w_beats_left == '0) & (bursts_out == '0);

   always_ff @(posedge clk) begin
      done <= 1'b0;

      // ---- AW ----
      if (m_axi_awready)
        m_axi_awvalid <= 1'b0;

      if (can_launch_aw) begin
         m_axi_awvalid <= 1'b1;
         m_axi_awaddr  <= aw_addr;
         m_axi_awlen   <= 8'(this_aw - 1'b1);
         aw_addr       <= aw_addr + (ADDR_WIDTH'(this_aw) << BEAT_LSB);
         aw_beats_left <= aw_beats_left - 32'(this_aw);
      end

      // outstanding write bursts (AW accepted .. B received)
      unique case ({aw_acc, b_acc})
        2'b10  : bursts_out <= bursts_out + 1'b1;
        2'b01  : bursts_out <= bursts_out - 1'b1;
        default: bursts_out <= bursts_out;
      endcase

      // ---- W ----
      if (w_start) begin
         w_active <= 1'b1;
         w_cnt    <= len_dout;
      end
      else if (w_acc) begin
         w_beats_left <= w_beats_left - 1'b1;
         if (w_cnt == 9'd1)
           w_active <= 1'b0;     // burst complete
         else
           w_cnt <= w_cnt - 1'b1;
      end

      // ---- job control ----
      if (start) begin
         running       <= 1'b1;
         aw_addr       <= {base[ADDR_WIDTH-1:BEAT_LSB], {BEAT_LSB{1'b0}}};
         aw_beats_left <= nbeats;
         w_beats_left  <= nbeats;
         bursts_out    <= '0;
      end
      else if (all_done) begin
         running <= 1'b0;
         done    <= 1'b1;
      end

      if (rst) begin
         running       <= 1'b0;
         aw_beats_left <= '0;
         w_beats_left  <= '0;
         bursts_out    <= '0;
         aw_addr       <= '0;
         m_axi_awvalid <= 1'b0;
         w_active      <= 1'b0;
         w_cnt         <= '0;
         done          <= 1'b0;
      end
   end

endmodule // vctrl_dma_wr
