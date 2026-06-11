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

   localparam REG_ADR_HIBIT = 7;

   localparam [REG_ADR_HIBIT : 2] CTRL_ADR  = 6'b00_0000;
   localparam [REG_ADR_HIBIT : 2] STAT_ADR  = 6'b00_0001;
   localparam [REG_ADR_HIBIT : 2] HTIM_ADR  = 6'b00_0010;
   localparam [REG_ADR_HIBIT : 2] VTIM_ADR  = 6'b00_0011;
   localparam [REG_ADR_HIBIT : 2] HVLEN_ADR = 6'b00_0100;
   localparam [REG_ADR_HIBIT : 2] VBAR_ADR  = 6'b00_0101;
   localparam [REG_ADR_HIBIT : 2] PITCH_ADR = 6'b00_1000;

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
