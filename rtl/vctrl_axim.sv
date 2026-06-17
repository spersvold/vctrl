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
    input  logic [ADDR_WIDTH-1:0]      fb_size,    // framebuffer size in bytes (bounds prefetch)
    input  logic                       ven,        // scanout enabled; gates prefetch off when 0
    output logic                       fetch_idle, // no reads in flight (safe to retire the buffer)
    input  logic                       frame_sys,  // start-of-frame (restart prefetch)
    input  logic                       fb_rdreq,
    input  logic [FB_ADDR_WIDTH-1:0]   fb_raddr,
    output logic                       fb_rdack,
    output logic [FB_DATA_WIDTH-1:0]   fb_rdata,
    output logic                       fb_rvalid,

    // ----------------------------------------------------------------------
    // AXI4 read-only master
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
    output logic                       m_axi_rready
    );

   localparam logic [1:0] BURST_INCR = 2'b01;

   // ========================================================================
   // Beat FIFO (holds prefetched AXI beats)
   // ========================================================================
   wire                                fifo_wr, fifo_rd, fifo_full, fifo_empty;
   wire [AXI_DATA_WIDTH-1:0]           fifo_dout;
   wire [FIFO_LGDEPTH:0]               fifo_fill;
   logic                               flush;     // draining for a frame restart

   sfifo #
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
   logic [ADDR_WIDTH:0]                fb_end_r;      // base+size latched (33-bit; prefetch bound)
   logic [CNTW-1:0]                    out_beats;     // beats requested but not yet returned
   logic                               armed;         // a frame has been started at least once
   logic                               ven_r;

   wire ar_acc = m_axi_arvalid & m_axi_arready;
   wire r_acc  = m_axi_rvalid  & m_axi_rready;

   // only push received beats into the FIFO when not flushing (else discard)
   assign fifo_wr = r_acc & ~flush;

   // issue a burst when armed, not flushing, and the FIFO has room for it
   // (counting beats already in flight)
   wire room_for_burst = (out_beats + {1'b0, fifo_fill} + CNTW'(2*BURST_LEN)) <= CNTW'(FIFO_DEPTH);

   // No reads outstanding to memory. After the consumer drops ven this falls
   // and stays low once the in-flight reads return, so software can wait on it
   // before the buffer's mapping is torn down.
   assign fetch_idle = (out_beats == '0);

   // Issue a burst only while scanout is enabled (ven). Gating on ven -- not
   // just letting the timing stop -- is what halts the read-ahead on disable:
   // otherwise the prefetcher keeps topping up the FIFO from the current buffer
   // and an in-flight read can fault when that buffer is unmapped. Also stop at
   // the end of the framebuffer (base + size): the read-ahead would otherwise
   // run a few bursts past the last line, which behind an IOMMU/SMMU hits an
   // unmapped page (on a flat map it silently read adjacent memory). 33-bit
   // compare so base+size at the top of the 4 GiB window does not wrap.

   wire ar_pending = m_axi_arvalid & ~m_axi_arready;
   wire can_launch = ven & armed & ~flush & room_for_burst &
                     ({1'b0, pf_addr} < fb_end_r) & ~ar_pending;

   assign m_axi_arid    = '0;
   assign m_axi_arlen   = 8'(BURST_LEN - 1);
   assign m_axi_arsize  = 3'(AXSIZE);
   assign m_axi_arburst = BURST_INCR;
   assign m_axi_arlock  = 1'b0;
   assign m_axi_arcache = 4'b0011;
   assign m_axi_arprot  = 3'b000;
   assign m_axi_rready  = 1'b1;

   // Detect rising and falling edges of VEN to re-capture fb_base/end
   wire ven_rise =  ven & ~ven_r;
   wire ven_fall = ~ven &  ven_r;

   always_ff @(posedge clk) begin
      // outstanding beat accounting
      unique case ({ar_acc, r_acc})
        2'b10  : out_beats <= out_beats + CNTW'(BURST_LEN);
        2'b01  : out_beats <= out_beats - 1'b1;
        2'b11  : out_beats <= out_beats + CNTW'(BURST_LEN) - 1'b1;
        default: out_beats <= out_beats;
      endcase

      ven_r <= ven;

      // clear pending request when ready
      if (m_axi_arready)
        m_axi_arvalid <= 1'b0;

      // AR request register: launch (latch address, advance pointer), then hold
      // the request unchanged until arready
      if (can_launch) begin
         m_axi_arvalid <= 1'b1;
         m_axi_araddr  <= pf_addr;
         pf_addr       <= pf_addr + ADDR_WIDTH'(BURST_LEN * STRB_WIDTH);
      end

      // frame restart: latch base, rewind prefetch, begin flush/drain
      if (frame_sys | ven_rise) begin
         armed     <= frame_sys;
         flush     <= 1'b1;
         fb_base_r <= fb_base;
         fb_end_r  <= {1'b0, fb_base} + {1'b0, fb_size};
         pf_addr   <= {fb_base[ADDR_WIDTH-1:BEAT_LSB], {BEAT_LSB{1'b0}}};
      end
      else if (flush & (out_beats == '0) & ~ar_pending) begin
         flush <= 1'b0;                 // drained; resume prefetch
      end

      // Disarm when VEN deasserts
      if (ven_fall)
        armed <= 1'b0;

      if (rst) begin
         ven_r     <= 1'b0;
         armed     <= 1'b0;
         flush     <= 1'b0;
         pf_addr   <= '0;
         fb_base_r <= '0;
         fb_end_r  <= '0;
         out_beats <= '0;
         m_axi_arvalid <= 1'b0;
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
   // Gate the pop on an actual request: without fb_rdreq, req_beat is stale
   // (e.g. fb_raddr=0 after the consumer is reset) so need_load would stick
   // high and drain the FIFO with no consumer -- which keeps room_for_burst
   // true and runs the prefetch pointer off the end of the framebuffer (and off
   // the end of memory), stalling the AXI read channel. Requiring fb_rdreq
   // bounds the prefetch to <= FIFO_DEPTH ahead of real consumption.
   assign fifo_rd = fb_rdreq & need_load & ~fifo_empty & ~flush;

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

endmodule // vctrl_axim
