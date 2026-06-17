// -*- mode: verilog; mode: font-lock; indent-tabs-mode: nil -*-
// vi: set et ts=3 sw=3 sts=3:
//
// Copyright 2021-2026 Steffen Persvold
// SPDX-License-Identifier: Apache-2.0
//
// ========================================================================
// File        : vctrl_pkg.sv
// Author      : Steffen Persvold
// Created     : April 15, 2021
// ========================================================================
// Description : Video Controller package with constants and typedefs
// ========================================================================
//

package vctrl_pkg;

   // Register address decode hi-bit. Set to 8 so byte offsets up to 0x1FC are
   // decoded -- the PLL reconfig CSRs live at 0x100/0x104, deliberately clear
   // of the OpenCores ocfb register map (which only occupies the low offsets;
   // anything in 0x0A0..0x7FF is free, 0x800+ is the CLUT region).
   localparam REG_ADR_HIBIT = 8;

   // OpenCores ocfb-compatible registers (byte offsets in comments)
   localparam [REG_ADR_HIBIT : 2] CTRL_ADR      = 7'b000_0000; // 0x000
   localparam [REG_ADR_HIBIT : 2] STAT_ADR      = 7'b000_0001; // 0x004
   localparam [REG_ADR_HIBIT : 2] HTIM_ADR      = 7'b000_0010; // 0x008
   localparam [REG_ADR_HIBIT : 2] VTIM_ADR      = 7'b000_0011; // 0x00C
   localparam [REG_ADR_HIBIT : 2] HVLEN_ADR     = 7'b000_0100; // 0x010
   localparam [REG_ADR_HIBIT : 2] VBAR_ADR      = 7'b000_0101; // 0x014
   localparam [REG_ADR_HIBIT : 2] VSIZ_ADR      = 7'b000_0110; // 0x018 - scanout buffer size (bounds the prefetch)
   localparam [REG_ADR_HIBIT : 2] PITCH_ADR     = 7'b000_1000; // 0x020

   // PLL reconfiguration CSRs (extension beyond the OpenCores ocfb register map)
   localparam [REG_ADR_HIBIT : 2] PLLDIVCNT_ADR = 7'b100_0000; // 0x100 - PLL M/N/C divisors
   localparam [REG_ADR_HIBIT : 2] PLLCTRL_ADR   = 7'b100_0001; // 0x104 - PLL reconfig trigger/status

   typedef enum logic [1:0] {
      BPP_8  = 2'd0,
      BPP_16 = 2'd1,
      BPP_24 = 2'd2,
      BPP_32 = 2'd3
   } bpp_t;

   // Timing registers
   typedef struct packed {
      logic [ 7:0] tsync;   // Sync pulse width -1
      logic [ 7:0] tgdel;   // Gate delay width -1
      logic [11:0] tgate;   // Gate width -1
      logic [11:0] tlen;    // Total length -1
   } timing_t;

   // Control register
   typedef struct packed {
      logic [31:17] rsvd1;  // reserved
      logic         dbl;    // doublescan mode
      logic         bl;     // blanking level
      logic         csl;    // composite sync level
      logic         vsl;    // vsync level
      logic         hsl;    // hsync level
      logic         pc;     // pseudo color
      bpp_t         cd;     // color depth
      logic [ 8:3]  rsvd2;  // reserved
      logic         hie;    // hsync interrupt enable
      logic         vie;    // vsync interrupt enable
      logic         ven;    // video system enable
   } ctrl_t;

   // PLL divisor register (PLLDIVCNT): logical M/N/C divide values.
   // Hardware (hdmi_pll_recfg) encodes these into the IOPLL counter fields.
   //   M : feedback / multiply total count (4..320)
   //   N : input divide                    (1..110)
   //   C : post-scale, drives clk_pix      (1..510)
   typedef struct packed {
      logic [ 6:0] rsvd;    // [31:25]
      logic [ 8:0] c;       // [24:16]
      logic [ 6:0] n;       // [15: 9]
      logic [ 8:0] m;       // [ 8: 0]
   } plldivcnt_t;

   // LB colour channels
   typedef struct packed {
      logic [7:0] red;
      logic [7:0] green;
      logic [7:0] blue;
   } lb_data_t;

   // Color index width (in bytes)
   function automatic logic [2:0] fb_cidxb(input bpp_t bpp);
      case (bpp)
        BPP_32 : return 3'd4;
        BPP_24 : return 3'd3;
        BPP_16 : return 3'd2;
        default: return 3'd1;
      endcase
   endfunction // fb_cidxb

   // ===================================================================
   // DMA engine (vctrl_dma) ABI
   //
   //   The DMA engine exposes its own register slave (cmd_*), a
   //   sibling of the core's cfg_* register bus using the same request/ack
   //   protocol. It moves data with two AXI masters: a read master that
   //   fetches from a source buffer and a write master that stores to the
   //   destination, signalling completion with a monotonic fence seqno and
   //   an interrupt. Datapath sizing (AXI width, burst length, FIFO depth)
   //   is carried as vctrl_dma module parameters, not here -- this is the
   //   software-visible contract only.
   // ===================================================================

   localparam logic [15:0] DMA_VERSION = 16'h0100;               // 1.0
   localparam logic [31:0] DMA_IDENT   = {16'h7643, DMA_VERSION};  // "vc" + version

   // CSR map -- cmd_* window. Word addresses; byte offsets in comments.
   localparam [REG_ADR_HIBIT : 2] DMA_ID_ADR           = 7'h00; // 0x00 RO   identification
   localparam [REG_ADR_HIBIT : 2] DMA_CTRL_ADR         = 7'h01; // 0x04 RW   enable / soft-reset
   localparam [REG_ADR_HIBIT : 2] DMA_STATUS_ADR       = 7'h02; // 0x08 RO   busy / idle / error / state
   localparam [REG_ADR_HIBIT : 2] DMA_IRQ_ADR          = 7'h03; // 0x0C RW1C interrupt status
   localparam [REG_ADR_HIBIT : 2] DMA_IRQEN_ADR        = 7'h04; // 0x10 RW   interrupt mask
   // PIO descriptor (Phase 1a direct submit; kept as a debug path)
   localparam [REG_ADR_HIBIT : 2] DMA_SRC_ADR          = 7'h08; // 0x20 RW   source byte address
   localparam [REG_ADR_HIBIT : 2] DMA_DST_ADR          = 7'h09; // 0x24 RW   destination byte offset
   localparam [REG_ADR_HIBIT : 2] DMA_SRCPITCH_ADR     = 7'h0A; // 0x28 RW   source row stride (bytes)
   localparam [REG_ADR_HIBIT : 2] DMA_DSTPITCH_ADR     = 7'h0B; // 0x2C RW   dest row stride (bytes)
   localparam [REG_ADR_HIBIT : 2] DMA_WIDTH_ADR        = 7'h0C; // 0x30 RW   bytes per row
   localparam [REG_ADR_HIBIT : 2] DMA_HEIGHT_ADR       = 7'h0D; // 0x34 RW   rows (1 => linear copy)
   localparam [REG_ADR_HIBIT : 2] DMA_OP_ADR           = 7'h0E; // 0x38 RW   [7:0] opcode, [15:8] flags
   localparam [REG_ADR_HIBIT : 2] DMA_DOORBELL_ADR     = 7'h0F; // 0x3C WO   write seqno -> execute descriptor
   // Fence / error
   localparam [REG_ADR_HIBIT : 2] DMA_FENCE_ADR        = 7'h10; // 0x40 RO   last completed seqno
   localparam [REG_ADR_HIBIT : 2] DMA_ERR_ADR          = 7'h11; // 0x44 RO   error info (resp/fault)
   // Ring submit (Phase 1b; reserved so the ABI is stable)
   localparam [REG_ADR_HIBIT : 2] DMA_RINGBASE_ADR     = 7'h14; // 0x50 RW   host ring base address
   localparam [REG_ADR_HIBIT : 2] DMA_RINGSIZE_ADR     = 7'h15; // 0x54 RW   log2(entries)
   localparam [REG_ADR_HIBIT : 2] DMA_RINGTAIL_ADR     = 7'h16; // 0x58 WO   producer doorbell (tail)
   localparam [REG_ADR_HIBIT : 2] DMA_RINGHEAD_ADR     = 7'h17; // 0x5C RO   consumer pointer (head)

   // DMA_CTRL bit positions
   localparam int DMA_CTRL_ENABLE = 0;   // engine enable
   localparam int DMA_CTRL_RESET  = 1;   // soft reset (self-clearing)
   localparam int DMA_CTRL_RINGEN = 2;   // process the in-memory command ring

   // DMA_STATUS bit positions ([11:4] = FSM state, debug)
   localparam int DMA_STAT_BUSY   = 0;   // a command is executing
   localparam int DMA_STAT_IDLE   = 1;   // engine idle, no in-flight beats
   localparam int DMA_STAT_ERROR  = 2;   // sticky error (see DMA_ERR)

   // DMA_IRQ / DMA_IRQEN bit positions (DMA_IRQ is write-1-to-clear)
   localparam int DMA_IRQ_DONE    = 0;   // command completed (fence advanced)
   localparam int DMA_IRQ_ERROR   = 1;   // command faulted

   // Opcodes (DMA_OP[7:0] / descriptor opflags[7:0])
   typedef enum logic [7:0] {
      DMA_OP_NOP    = 8'd0,
      DMA_OP_COPY2D = 8'd1,   // 2D strided copy source -> destination (linear = height 1)
      DMA_OP_FILL   = 8'd2,   // solid fill of destination (src_addr = pattern) [Phase 1.5]
      DMA_OP_FENCE  = 8'd3    // advance fence + raise IRQ, no data movement
   } dma_opcode_t;

   // Command descriptor (8 words = 32 bytes). Same layout for the PIO
   // registers and the in-memory ring; opflags is the least-significant
   // word (word0, lowest byte address in the ring):
   //   word0 opflags  [7:0] opcode, [15:8] flags
   //   word1 seqno    fence value reported on completion
   //   word2 src_addr source byte address (read master)
   //   word3 dst_addr destination byte offset (write master)
   //   word4 src_pitch / word5 dst_pitch  row strides (bytes)
   //   word6 width    bytes per row
   //   word7 height   number of rows (1 => linear copy)
   typedef struct packed {
      logic [31:0] height;      // word7  [255:224]
      logic [31:0] width;       // word6
      logic [31:0] dst_pitch;   // word5
      logic [31:0] src_pitch;   // word4
      logic [31:0] dst_addr;    // word3
      logic [31:0] src_addr;    // word2
      logic [31:0] seqno;       // word1
      logic [31:0] opflags;     // word0  [31:0]
   } dma_desc_t;

   localparam int DMA_DESC_WORDS = 8;

endpackage : vctrl_pkg
