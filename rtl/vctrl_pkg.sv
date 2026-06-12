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

endpackage : vctrl_pkg
