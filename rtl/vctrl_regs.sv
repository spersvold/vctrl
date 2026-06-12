// -*- mode: verilog; mode: font-lock; indent-tabs-mode: nil -*-
// vi: set et ts=3 sw=3 sts=3:
//
// Copyright 2021-2026 Steffen Persvold
// SPDX-License-Identifier: Apache-2.0
//
// ========================================================================
// File        : vctrl_regs.sv
// Author      : Steffen Persvold
// Created     : April 15, 2021
// ========================================================================
// Description : Video Controller CSRs
// ========================================================================
//

module vctrl_regs import vctrl_pkg::*;
  (
   input  logic         clk_sys,      // System clock
   input  logic         rst_sys,      // System Reset Signal

   input  logic         cfg_req,
   input  logic [11: 2] cfg_adr,
   input  logic         cfg_we,
   input  logic [ 3: 0] cfg_be,
   input  logic [31: 0] cfg_d,
   output logic [31: 0] cfg_q,
   output logic         cfg_ack,

   output logic         irq,

   // control register settings
   output ctrl_t        ctrl,

   // status register inputs
   input  logic         hint_in,      // hsync interrupt request
   input  logic         vint_in,      // vsync interrupt request

   // Horizontal Timing Register
   output timing_t      htim,

   // Vertical Timing Register
   output timing_t      vtim,

   // Horizontal line pitch
   output logic [11:0]  pitch,

   // color lookup table signals
   output logic         clut_req,
   input  logic         clut_ack,
   input  logic [23:0]  clut_q,

   // video base address (framebuffer scanout base, word aligned)
   output logic [31:2]  vbar,

   // PLL reconfiguration interface (to/from hdmi_pll_recfg).
   //   pll_divcnt : PLLDIVCNT contents (logical M/N/C), quasi-static
   //   pll_apply  : 1-cycle trigger pulse (clk_sys) on PLLCTRL.apply W1S write
   //   pll_done   : 1-cycle completion pulse (clk_sys, CDC'd from recfg)
   //   pll_locked : synchronized PLL locked status
   //   pll_error  : synchronized recal-timeout/failure status
   output plldivcnt_t   pll_divcnt,
   output logic         pll_apply,
   input  logic         pll_done,
   input  logic         pll_locked,
   input  logic         pll_error
   );

   //
   // variable declarations
   //

   wire [REG_ADR_HIBIT : 2] reg_adr  = cfg_adr[REG_ADR_HIBIT : 2];

   logic                  hint, vint;
   logic                  acc, acc32, reg_acc, reg_wacc;
   logic [31:0]           reg_dato; // data output from registers

   logic                  pll_busy;     // PLLCTRL.apply read-back (busy while reconfig in flight)

   //
   // Module body
   //

   assign clut_req =  cfg_adr[11] & cfg_req;
   assign reg_acc  = ~cfg_adr[11] & cfg_req;
   assign reg_wacc =  reg_acc & cfg_we;

   always_ff @(posedge clk_sys)
     cfg_ack <= (reg_acc | clut_ack) & ~cfg_ack;

   // generate registers
   always_ff @(posedge clk_sys)
     if (rst_sys)
       begin
          htim <= '0;
          vtim <= '0;
          vbar <= '0;
       end
     else if (reg_wacc)
       unique case (reg_adr)
         HTIM_ADR  : begin
            htim.tsync <= cfg_d[31:24];
            htim.tgdel <= cfg_d[23:16];
            htim.tgate <= cfg_d[11: 0];
         end
         VTIM_ADR  : begin
            vtim.tsync <= cfg_d[31:24];
            vtim.tgdel <= cfg_d[23:16];
            vtim.tgate <= cfg_d[11: 0];
         end
         HVLEN_ADR : begin
            htim.tlen  <= cfg_d[27:16];
            vtim.tlen  <= cfg_d[11: 0];
         end
         VBAR_ADR  : begin
            vbar       <= cfg_d[31: 2];
         end
         PITCH_ADR : begin
            pitch      <= cfg_d[11: 0];
         end
         default:;
       endcase

   // generate control register
   always_ff @(posedge clk_sys)
     if (rst_sys)
       ctrl <= '0;
     else if (reg_wacc & (reg_adr == CTRL_ADR))
       begin
          ctrl.dbl   <= cfg_d[16];
          ctrl.bl    <= cfg_d[15];
          ctrl.csl   <= cfg_d[14];
          ctrl.vsl   <= cfg_d[13];
          ctrl.hsl   <= cfg_d[12];
          ctrl.pc    <= cfg_d[11];
          ctrl.cd    <= bpp_t'(cfg_d[10:9]);
          ctrl.hie   <= cfg_d[2];
          ctrl.vie   <= cfg_d[1];
          ctrl.ven   <= cfg_d[0];
       end

   // generate status register
   always_ff @(posedge clk_sys)
     if (rst_sys)
       begin
          hint <= 1'b0;
          vint <= 1'b0;
       end
     else
       begin
          if (reg_wacc & (reg_adr == STAT_ADR) )
            begin
               hint <= hint_in | (hint & ~cfg_d[5]);
               vint <= vint_in | (vint & ~cfg_d[4]);
            end
          else
            begin
               hint <= hint | hint_in;
               vint <= vint | vint_in;
            end
       end

   // assign output
   always_comb
     unique casez (reg_adr)
       CTRL_ADR  : reg_dato = ctrl;
       STAT_ADR  : reg_dato = {26'h0, hint, vint, 4'h0};
       HTIM_ADR  : begin
          reg_dato[31:24] = htim.tsync;
          reg_dato[23:16] = htim.tgdel;
          reg_dato[15: 0] = {4'h0, htim.tgate};
       end
       VTIM_ADR  : begin
          reg_dato[31:24] = vtim.tsync;
          reg_dato[23:16] = vtim.tgdel;
          reg_dato[15: 0] = {4'h0, vtim.tgate};
       end
       HVLEN_ADR : begin
          reg_dato[31:16] = {4'h0, htim.tlen};
          reg_dato[15: 0] = {4'h0, vtim.tlen};
       end
       VBAR_ADR  : reg_dato = {vbar, 2'b0};
       PLLDIVCNT_ADR : reg_dato = pll_divcnt;
       PLLCTRL_ADR   : reg_dato = {29'h0, pll_error, pll_locked, pll_busy};
       PITCH_ADR : reg_dato = {20'h0, pitch};
       default   : reg_dato = 32'h0000_0000;
     endcase

   always_ff @(posedge clk_sys)
     cfg_q <= reg_acc ? reg_dato : {8'h0, clut_q};

   // PLL divisor register (PLLDIVCNT). Latched only while no reconfig is in
   // flight so an in-progress sequence cannot be corrupted mid-flight.
   always_ff @(posedge clk_sys)
     if (rst_sys)
       pll_divcnt <= '0;
     else if (reg_wacc & (reg_adr == PLLDIVCNT_ADR) & ~pll_busy)
       pll_divcnt <= cfg_d;

   // PLLCTRL.apply: W1S trigger with busy read-back. Writing 1 (when idle)
   // emits a one-cycle pll_apply pulse and sets busy; busy clears when the
   // reconfig FSM returns its done pulse. Writes ignored while busy.
   always_ff @(posedge clk_sys)
     if (rst_sys)
       begin
          pll_busy  <= 1'b0;
          pll_apply <= 1'b0;
       end
     else
       begin
          pll_apply <= 1'b0;
          if (reg_wacc & (reg_adr == PLLCTRL_ADR) & cfg_d[0] & ~pll_busy)
            begin
               pll_busy  <= 1'b1;
               pll_apply <= 1'b1;
            end
          else if (pll_done)
            pll_busy <= 1'b0;
       end

   // generate interrupt request signal
   always_ff @(posedge clk_sys)
     irq <= (hint & ctrl.hie) |
            (vint & ctrl.vie);

endmodule // vctrl_regs
