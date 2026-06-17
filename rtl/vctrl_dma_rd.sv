// -*- mode: verilog; mode: font-lock; indent-tabs-mode: nil -*-
// vi: set et ts=3 sw=3 sts=3:
//
// Copyright 2026 Steffen Persvold
// SPDX-License-Identifier: Apache-2.0
//
// ========================================================================
// File        : vctrl_dma_rd.sv
// Author      : Steffen Persvold
// Created     : June 17, 2026
// ========================================================================
// Description : DMA source read master.
//
//   Reads a contiguous run of `nbeats` full-width beats starting at `base`
//   and streams them into the copy FIFO (held in the parent). Issues long
//   INCR bursts (up to BURST_LEN beats; the final burst is short when the
//   run is not a burst multiple), kept many-outstanding so the source
//   round-trip latency is hidden -- the property that makes a DMA fill beat
//   the CPU's serialized single-beat writes.
//
//   The prefetch reuses the scanout master's hardened pattern: arvalid is
//   registered and held until arready (never retracted), the pointer
//   advances at launch, and room_for_burst reserves 2*BURST_LEN so the
//   registration lag cannot over-commit the FIFO. The R data bus is wired
//   straight to the FIFO in the parent; this block only counts beats and
//   drives the read-address channel + rready.
// ========================================================================
//

module vctrl_dma_rd
  #(
    parameter  integer ADDR_WIDTH     = 32,
    parameter  integer AXI_DATA_WIDTH = 256,
    parameter  integer ID_WIDTH       = 1,
    parameter  integer BURST_LEN      = 16,    // max beats per AR
    parameter  integer FIFO_LGDEPTH   = 9,     // copy FIFO depth = 2**FIFO_LGDEPTH
    localparam integer STRB_WIDTH     = AXI_DATA_WIDTH/8,
    localparam integer AXSIZE         = $clog2(STRB_WIDTH),
    localparam integer BEAT_LSB       = $clog2(STRB_WIDTH),     // byte->beat shift
    localparam integer FIFO_DEPTH     = (1 << FIFO_LGDEPTH),
    localparam integer CNTW           = FIFO_LGDEPTH + 2
    )
   (
    input  logic                       clk,
    input  logic                       rst,

    // ----------------------------------------------------------------------
    // Job control
    // ----------------------------------------------------------------------
    input  logic                       start,   // one-cycle: begin a transfer
    input  logic [ADDR_WIDTH-1:0]      base,    // source byte address (beat aligned)
    input  logic [31:0]                nbeats,  // number of beats to read
    output logic                       busy,    // transfer in progress
    output logic                       done,    // one-cycle: all beats returned

    // ----------------------------------------------------------------------
    // Copy FIFO occupancy (FIFO lives in the parent; R data wired to it there)
    // ----------------------------------------------------------------------
    input  logic [FIFO_LGDEPTH:0]      fifo_fill,

    // ----------------------------------------------------------------------
    // AXI4 read master (address + read-data handshake)
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
    input  logic                       m_axi_rvalid,
    input  logic                       m_axi_rlast,
    output logic                       m_axi_rready
    );

   localparam logic [1:0] BURST_INCR = 2'b01;

   logic [ADDR_WIDTH-1:0] cur_addr;     // next burst address
   logic [31:0]           beats_left;   // beats still to request
   logic [CNTW-1:0]       out_beats;    // requested but not yet returned
   logic                  running;

   wire ar_acc     = m_axi_arvalid & m_axi_arready;
   wire r_acc      = m_axi_rvalid  & m_axi_rready;
   wire ar_pending = m_axi_arvalid & ~m_axi_arready;

   // Beats in the burst we are about to launch (short final burst).
   wire [8:0] this_burst = (beats_left >= BURST_LEN) ? 9'(BURST_LEN) : beats_left[8:0];

   // Beats just accepted = the held arlen+1.
   wire [CNTW-1:0] acc_beats = CNTW'(m_axi_arlen) + 1'b1;

   // Room for one more burst, counting in-flight + buffered + a 2*BURST_LEN
   // reserve for the registration lag (the hardened scanout idiom).
   wire room_for_burst = (out_beats + {1'b0, fifo_fill} + CNTW'(2*BURST_LEN)) <= CNTW'(FIFO_DEPTH);

   wire all_done   = running & (beats_left == '0) & (out_beats == '0);
   wire can_launch = running & (beats_left != '0) & room_for_burst & ~ar_pending;

   assign busy          = running;

   assign m_axi_arid    = '0;
   assign m_axi_arsize  = 3'(AXSIZE);
   assign m_axi_arburst = BURST_INCR;
   assign m_axi_arlock  = 1'b0;
   assign m_axi_arcache = 4'b0011;
   assign m_axi_arprot  = 3'b000;
   assign m_axi_rready  = 1'b1;     // room_for_burst guarantees FIFO space

   always_ff @(posedge clk) begin
      done <= 1'b0;

      // outstanding beat accounting
      unique case ({ar_acc, r_acc})
        2'b10  : out_beats <= out_beats + acc_beats;
        2'b01  : out_beats <= out_beats - 1'b1;
        2'b11  : out_beats <= out_beats + acc_beats - 1'b1;
        default: out_beats <= out_beats;
      endcase

      // clear the request once accepted
      if (m_axi_arready)
        m_axi_arvalid <= 1'b0;

      // launch a burst: latch address/len, advance pointer (a same-cycle
      // accept relaunches because this assignment wins over the clear above)
      if (can_launch) begin
         m_axi_arvalid <= 1'b1;
         m_axi_araddr  <= cur_addr;
         m_axi_arlen   <= 8'(this_burst - 1'b1);
         cur_addr      <= cur_addr + (ADDR_WIDTH'(this_burst) << BEAT_LSB);
         beats_left    <= beats_left - 32'(this_burst);
      end

      if (start) begin
         running    <= 1'b1;
         cur_addr   <= {base[ADDR_WIDTH-1:BEAT_LSB], {BEAT_LSB{1'b0}}};
         beats_left <= nbeats;
         out_beats  <= '0;
      end
      else if (all_done) begin
         running <= 1'b0;
         done    <= 1'b1;
      end

      if (rst) begin
         running       <= 1'b0;
         beats_left    <= '0;
         out_beats     <= '0;
         cur_addr      <= '0;
         m_axi_arvalid <= 1'b0;
         done          <= 1'b0;
      end
   end

endmodule // vctrl_dma_rd
