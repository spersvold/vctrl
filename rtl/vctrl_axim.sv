// -*- mode: verilog; mode: font-lock; indent-tabs-mode: nil -*-
// vi: set et ts=3 sw=3 sts=3:
//
// Copyright 2026 Steffen Persvold
// SPDX-License-Identifier: Apache-2.0
//
// ========================================================================
// File        : vctrl_axim.sv
// Author      : Steffen Persvold
// ========================================================================
// Description : Video controller scanout AXI read master with line buffer.
//
//   Bridges the video controller's framebuffer read port (the custom
//   fb_rdreq / fb_raddr / fb_rdack / fb_rdata / fb_rvalid split read bus,
//   FB_DATA_WIDTH words) onto a native AXI_DATA_WIDTH AXI4 read master, so
//   the width conversion lives here and no external adapter is needed.
//
//   A prefetcher issues long INCR bursts of full-width beats from a running
//   pointer into a beat FIFO, kept ahead of consumption. The serve front-end
//   holds the current beat and answers each fb request from the word at
//   byte_addr[.. : log2(FB bytes)] within it, so all words of a beat (and the
//   sub-pixel re-reads the controller issues for <32bpp) are served from the
//   buffer; the FIFO only advances when a request crosses to the next beat.
//   That word extraction is the AXI->fb width conversion.
//
//   Scanline layout is assumed contiguous (pitch == active width), so the
//   per-frame access is one sequential stream from fb_base. frame_sys resets
//   the prefetch pointer to (the latched) fb_base and flushes the FIFO,
//   draining any in-flight reads first; this is expected to happen during
//   vertical blanking. The write channel is unused (read-only) and idle.
// ========================================================================
//

module vctrl_axim
  #(
    parameter  integer ADDR_WIDTH     = 32,    // AXI address width
    parameter  integer AXI_DATA_WIDTH = 256,   // AXI/memory data width
    parameter  integer FB_DATA_WIDTH  = 32,    // controller word width (VR_DATAW)
    parameter  integer FB_ADDR_WIDTH  = 24,    // fb_raddr width (framebuffer-relative)
    parameter  integer ID_WIDTH       = 4,
    parameter  integer BURST_LEN      = 8,     // beats per prefetch AR
    parameter  integer FIFO_LGDEPTH   = 6,     // beat FIFO depth = 2**FIFO_LGDEPTH
    localparam integer STRB_WIDTH     = AXI_DATA_WIDTH/8,
    localparam integer AXSIZE         = $clog2(STRB_WIDTH),
    localparam integer WORDS_PER_BEAT = AXI_DATA_WIDTH/FB_DATA_WIDTH,
    localparam integer WORD_LSB       = $clog2(FB_DATA_WIDTH/8),   // byte->word shift
    localparam integer BEAT_LSB       = $clog2(STRB_WIDTH),        // byte->beat shift
    localparam integer FIFO_DEPTH     = (1 << FIFO_LGDEPTH),
    localparam integer CNTW           = FIFO_LGDEPTH + 2
    )
   (
    input  logic                       clk,
    input  logic                       rst,

    // ----------------------------------------------------------------------
    // Video controller framebuffer read port (slave, read-only)
    // ----------------------------------------------------------------------
    input  logic [ADDR_WIDTH-1:0]      fb_base,    // framebuffer base address
    input  logic                       frame_sys,  // start-of-frame (restart prefetch)
    input  logic                       fb_rdreq,
    input  logic [FB_ADDR_WIDTH-1:0]   fb_raddr,
    output logic                       fb_rdack,
    output logic [FB_DATA_WIDTH-1:0]   fb_rdata,
    output logic                       fb_rvalid,

    // ----------------------------------------------------------------------
    // AXI4 read master (write channel tied idle)
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
    //
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

   // ========================================================================
   // Beat FIFO (holds prefetched AXI beats)
   // ========================================================================
   wire                                fifo_wr, fifo_rd, fifo_full, fifo_empty;
   wire [AXI_DATA_WIDTH-1:0]           fifo_dout;
   wire [FIFO_LGDEPTH:0]               fifo_fill;
   logic                               flush;     // draining for a frame restart

   scfifo #
     (.WIDTH (AXI_DATA_WIDTH),
      .LGSIZ (FIFO_LGDEPTH))
   u_fifo
     (.clk,
      .rst   (rst | flush),          // flush empties the FIFO on restart
      .wen   (fifo_wr),
      .din   (m_axi_rdata),
      .full  (fifo_full),
      .fill  (fifo_fill),
      .ren   (fifo_rd),
      .dout  (fifo_dout),
      .empty (fifo_empty));

   // ========================================================================
   // Prefetch engine : sequential bursts into the FIFO
   // ========================================================================
   logic [ADDR_WIDTH-1:0]              pf_addr;       // next burst address (beat aligned)
   logic [ADDR_WIDTH-1:0]              fb_base_r;     // base latched for the current frame
   logic [CNTW-1:0]                    out_beats;     // beats requested but not yet returned
   logic                               armed;         // a frame has been started at least once

   wire ar_acc = m_axi_arvalid & m_axi_arready;
   wire r_acc  = m_axi_rvalid  & m_axi_rready;

   // only push received beats into the FIFO when not flushing (else discard)
   assign fifo_wr = r_acc & ~flush;

   // issue a burst when armed, not flushing, and the FIFO has room for it
   // (counting beats already in flight)
   wire room_for_burst = (out_beats + {1'b0, fifo_fill} + CNTW'(BURST_LEN)) <= CNTW'(FIFO_DEPTH);

   assign m_axi_arvalid = armed & ~flush & room_for_burst;
   assign m_axi_araddr  = pf_addr;
   assign m_axi_arid    = '0;
   assign m_axi_arlen   = 8'(BURST_LEN - 1);
   assign m_axi_arsize  = 3'(AXSIZE);
   assign m_axi_arburst = BURST_INCR;
   assign m_axi_arlock  = 1'b0;
   assign m_axi_arcache = 4'b0011;
   assign m_axi_arprot  = 3'b000;
   assign m_axi_rready  = 1'b1;

   always_ff @(posedge clk) begin
      // outstanding beat accounting
      unique case ({ar_acc, r_acc})
        2'b10  : out_beats <= out_beats + CNTW'(BURST_LEN);
        2'b01  : out_beats <= out_beats - 1'b1;
        2'b11  : out_beats <= out_beats + CNTW'(BURST_LEN) - 1'b1;
        default: out_beats <= out_beats;
      endcase

      if (ar_acc)
        pf_addr <= pf_addr + ADDR_WIDTH'(BURST_LEN * STRB_WIDTH);

      // frame restart: latch base, rewind prefetch, begin flush/drain
      if (frame_sys) begin
         armed     <= 1'b1;
         flush     <= 1'b1;
         fb_base_r <= fb_base;
         pf_addr   <= {fb_base[ADDR_WIDTH-1:BEAT_LSB], {BEAT_LSB{1'b0}}};
      end
      else if (flush & (out_beats == '0)) begin
         flush <= 1'b0;                 // drained; resume prefetch
      end

      if (rst) begin
         armed     <= 1'b0;
         flush     <= 1'b0;
         pf_addr   <= '0;
         fb_base_r <= '0;
         out_beats <= '0;
      end
   end

   // ========================================================================
   // Serve front-end : extract the requested word from the current beat
   // ========================================================================
   logic [AXI_DATA_WIDTH-1:0] beat;       // current beat being served
   logic [ADDR_WIDTH-1:0]     beat_addr;  // beat-aligned address of `beat`
   logic [ADDR_WIDTH-1:0]     next_addr;  // beat-aligned address of next FIFO pop
   logic                      beat_valid;

   wire [ADDR_WIDTH-1:0] req_byte = fb_base_r + ADDR_WIDTH'(fb_raddr);
   wire [ADDR_WIDTH-1:0] req_beat = {req_byte[ADDR_WIDTH-1:BEAT_LSB], {BEAT_LSB{1'b0}}};
   wire [$clog2(WORDS_PER_BEAT)-1:0] req_word_off = req_byte[BEAT_LSB-1:WORD_LSB];

   wire hit       = beat_valid & (req_beat == beat_addr);
   wire need_load = ~beat_valid | (req_beat != beat_addr);   // first access of a new beat
   assign fifo_rd = need_load & ~fifo_empty & ~flush;

   assign fb_rdack = fb_rdreq & hit & ~flush;

   always_ff @(posedge clk) begin
      if (fifo_rd) begin
         beat       <= fifo_dout;
         beat_addr  <= next_addr;
         next_addr  <= next_addr + ADDR_WIDTH'(STRB_WIDTH);
         beat_valid <= 1'b1;
      end

      // registered fb response (one rvalid per rdack, 1-cycle latency)
      fb_rvalid <= fb_rdack;
      if (fb_rdack)
        fb_rdata <= beat[req_word_off*FB_DATA_WIDTH +: FB_DATA_WIDTH];

      if (frame_sys) begin
         beat_valid <= 1'b0;
         next_addr  <= {fb_base[ADDR_WIDTH-1:BEAT_LSB], {BEAT_LSB{1'b0}}};
         fb_rvalid  <= 1'b0;
      end

      if (rst) begin
         beat_valid <= 1'b0;
         next_addr  <= '0;
         fb_rvalid  <= 1'b0;
      end
   end

   // ========================================================================
   // Write channel : unused (read-only master)
   // ========================================================================
   assign m_axi_awid    = '0;
   assign m_axi_awaddr  = '0;
   assign m_axi_awlen   = '0;
   assign m_axi_awsize  = '0;
   assign m_axi_awburst = '0;
   assign m_axi_awlock  = 1'b0;
   assign m_axi_awcache = '0;
   assign m_axi_awprot  = '0;
   assign m_axi_awvalid = 1'b0;
   assign m_axi_wdata   = '0;
   assign m_axi_wstrb   = '0;
   assign m_axi_wlast   = 1'b0;
   assign m_axi_wvalid  = 1'b0;
   assign m_axi_bready  = 1'b1;

endmodule // vctrl_axim
