// -*- mode: verilog; mode: font-lock; indent-tabs-mode: nil -*-
// vi: set et ts=3 sw=3 sts=3:
//
// Copyright 2021-2026 Steffen Persvold
// SPDX-License-Identifier: Apache-2.0
//
// ========================================================================
// File        : vctrl_timing.sv
// Author      : Steffen Persvold
// Created     : April 15, 2021
// ========================================================================
// Description : Video Controller timing module
// ========================================================================
//

module vctrl_timing import vctrl_pkg::*;
   (
    input  logic             clk_pix,      // Pixel clock
    input  logic             rst_pix,      // Video reset

    input  timing_t          htim,         // horizontal timing settings
    input  timing_t          vtim,         // vertical timing settings inputs

    input  logic             hspol,        // horizontal sync pulse polarization level (pos/neg)
    input  logic             vspol,        // vertical sync pulse polarization level (pos/neg)

    input  logic             is_dbl,       // In doublescan mode

    // Outputs
    output logic             hsync,        // horizontal sync
    output logic             vsync,        // vertical sync
    output logic             frame,        // start of frame
    output logic             line,         // start of line
    output logic             gate          // data gate (low in blanking interval)
    );


   //

   typedef logic [$bits(htim.tgate)-1:0] hpos_t;

   hpos_t                    thsync;
   hpos_t                    thgdel;
   hpos_t                    thgate;
   hpos_t                    thlen;

   hpos_t                    sh2;
   hpos_t                    sh1;
   hpos_t                    hr;
   hpos_t                    hpos;

   typedef logic [$bits(vtim.tgate)-1:0] vpos_t;

   vpos_t                    tvsync;
   vpos_t                    tvgdel;
   vpos_t                    tvgate;
   vpos_t                    tvlen;

   vpos_t                    sv2;
   vpos_t                    sv1;
   vpos_t                    vr;
   vpos_t                    vpos;

   logic                     clk_en;

   assign thsync = hpos_t'(htim.tsync);
   assign thgdel = hpos_t'(htim.tgdel);
   assign thgate = hpos_t'(htim.tgate);
   assign thlen  = hpos_t'(htim.tlen);

   // double vertical timing values if scan doubling is enabled
   assign tvsync = (is_dbl) ? (((vpos_t'(vtim.tsync) + vpos_t'(1))<<1)-vpos_t'(1)) : vpos_t'(vtim.tsync);
   assign tvgdel = (is_dbl) ? (((vpos_t'(vtim.tgdel) + vpos_t'(1))<<1)-vpos_t'(1)) : vpos_t'(vtim.tgdel);
   assign tvgate = (is_dbl) ? (((vpos_t'(vtim.tgate) + vpos_t'(1))<<1)-vpos_t'(1)) : vpos_t'(vtim.tgate);
   assign tvlen  = (is_dbl) ? (((vpos_t'(vtim.tlen)  + vpos_t'(1))<<1)-vpos_t'(1)) : vpos_t'(vtim.tlen);

   // calculate absolute positions for sync start and end
   assign sh2 = thlen - thgdel;
   assign sh1 = sh2 - thsync - hpos_t'(1);
   assign hr  = thgate + hpos_t'(1); // active end, front porch start

   assign sv2 = tvlen - tvgdel;
   assign sv1 = sv2 - tvsync - vpos_t'(1);
   assign vr  = tvgate + vpos_t'(1); // active end, front porch start

   always_ff @(posedge clk_pix)
     if (rst_pix) clk_en <= 1'b1;
     else         clk_en <= (is_dbl) ? ~clk_en : clk_en; // in doublescan mode, count pixels half as fast

   always_ff @(posedge clk_pix)
     if (rst_pix) hpos <= hr; // in reset start at front-porch
     else if (clk_en)
       if (hpos == thlen)
         hpos <= '0;
       else
         hpos <= hpos + hpos_t'(1);

   wire clk_en_v = clk_en & (hpos == thgate); // enable vertical counter at end of active horizontal
   always_ff @(posedge clk_pix)
     if (rst_pix) vpos <= vr; // in reset start at front-porch
     else if (clk_en_v)
       if (vpos == tvlen)
         vpos <= '0;
       else
         vpos <= vpos + vpos_t'(1);

   // generate horizontal and vertical syncs with correct polarity
   always_ff @(posedge clk_pix)
     hsync <= ((hpos >= sh1) & (hpos <  sh2)) ^ hspol;

   always_ff @(posedge clk_pix)
     vsync <= ((vpos >= sv1) & (vpos <  sv2)) ^ vspol;

   // control signals
   always_ff @(posedge clk_pix)
     if (rst_pix) frame <= 1'b0;
     else         frame <= ((hpos == hr)  & (vpos == vr));

   always_ff @(posedge clk_pix)
     if (rst_pix) line  <= 1'b0;
     else         line  <= ((hpos == hr)  & (vpos <  vr));

   always_ff @(posedge clk_pix)
     if (rst_pix) gate  <= 1'b0;
     else         gate  <= ((hpos <  hr)  & (vpos <  vr));

endmodule // vctrl_timing
