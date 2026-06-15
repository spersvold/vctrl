// -*- mode: verilog; mode: font-lock; indent-tabs-mode: nil -*-
// vi: set et ts=3 sw=3 sts=3:
//
// Copyright 2021-2026 Steffen Persvold
// SPDX-License-Identifier: Apache-2.0
//
// ========================================================================
// File        : vctrl_core.sv
// Author      : Steffen Persvold
// Created     : April 15, 2021
// ========================================================================
// Description : Video Controller core
// ========================================================================
//

module vctrl_core import vctrl_pkg::*;
  #(
    parameter  integer VR_SIZE  = 640 * 480 * 2, // Size of Frame Buffer memory (bytes)
    parameter  integer VR_DATAW = 32,            // Frame Buffer Data Width (bits)
    parameter  integer LB_DEPTH = 1024,          // Line Buffer depth (in pixels), i.e this is effectively
                                                 // your max horizontal resolution
    localparam integer VR_ADDRW = $clog2(VR_SIZE)
    )
   (
    input  logic                   clk_sys,      // System clock
    input  logic                   rst_sys,      // System Reset Signal

    input  logic                   cfg_req,      // Config request
    input  logic [           11:2] cfg_adr,      // Config request address
    input  logic                   cfg_we,       // Config request write enable
    input  logic [            3:0] cfg_be,       // Config request byte enable
    input  logic [           31:0] cfg_d,        // Config request write data
    output logic [           31:0] cfg_q,        // Config request read data
    output logic                   cfg_ack,      // Config request acknowledge

    output logic                   irq,          // Interrupt
    output logic                   frame_sys,    // Start of new frame (in system clock domain)
    output logic [31:2]            vbar,         // Video base address (scanout base)
    output logic [31:2]            vsiz,         // Scanout buffer size (bytes); bounds the fetch
    output logic                   ven,          // Scanout enable (CTRL.VEN); gates the fetch
    input  logic                   fetch_idle,   // Scanout fetch idle (no reads in flight)

    output logic                   fb_rdreq,     // Memory read request
    output logic [VR_ADDRW   -1:0] fb_raddr,     // Memory read address
    input  logic                   fb_rdack,     // Memory read ackowledge
    input  logic [VR_DATAW   -1:0] fb_rdata,     // Memory read data
    input  logic                   fb_rvalid,    // Memory read data valid

    input  logic                   clk_pix,      // Pixel clock
    input  logic                   rst_pix,      // Pixel-domain reset (board-generated, async-assert/sync-deassert)

    // PLL reconfiguration interface (to/from board-level hdmi_pll_recfg)
    output plldivcnt_t             pll_divcnt,   // PLLDIVCNT (logical M/N/C)
    output logic                   pll_apply,    // reconfig trigger pulse (clk_sys)
    input  logic                   pll_done,     // reconfig done pulse (clk_sys)
    input  logic                   pll_locked,   // synchronized PLL locked
    input  logic                   pll_error,    // synchronized recal error

    output logic [            7:0] vga_r,        // VGA Red output
    output logic [            7:0] vga_g,        // VGA Green output
    output logic [            7:0] vga_b,        // VGA Blue output
    output logic                   vga_bl,       // VGA Blanking
    output logic                   vga_hs,       // VGA Horizontal sync
    output logic                   vga_vs        // VGA Vertical sync
    );

   // ========================================================================
   // Internal Nets
   // ========================================================================

   // Control register settings
   ctrl_t                          ctrl;

   // Horizontal Timing Register
   timing_t                        htim;

   // Vertical Timing Register
   timing_t                        vtim;

   // Horizontal line pitch
   logic [11:0]                    pitch;

   // Color LookUp Table signals
   logic                           cfg_clut_req;
   logic                           cfg_clut_ack;
   logic [23:0]                    cfg_clut_q;

   logic                           cp_clut_req;
   logic [ 7:0]                    cp_clut_adr;
   logic [23:0]                    cp_clut_q;
   logic                           cp_clut_ack;

   // Signals from timing module to framebuffer
   logic                           hsync, vsync,
                                   gate, frame, line;

   logic                           ven_pix;      // VEN synchronized into the pixel domain (scan enable)

   // ========================================================================
   // ========================================================================

   // Registers
   vctrl_regs u_regs
     (.clk_sys,
      .rst_sys,
      .cfg_req,
      .cfg_adr,
      .cfg_we,
      .cfg_be,
      .cfg_d,
      .cfg_q,
      .cfg_ack,
      .irq,
      .ctrl,
      .hint_in   (1'b0 /*vga_hs*/),
      // frame_sys is the start-of-frame pulse already CDC'd into clk_sys
      // (cdc_tgl in vctrl_fbuff) -- a clean once-per-frame vsync interrupt
      // request for KMS vblank. It coincides with vctrl_axim latching the
      // scanout base, so the IRQ marks exactly when a flipped buffer goes live.
      .vint_in   (frame_sys),
      .fetch_idle,
      .htim,
      .vtim,
      .pitch,
      .clut_req  (cfg_clut_req),
      .clut_ack  (cfg_clut_ack),
      .clut_q    (cfg_clut_q),
      .vbar,
      .vsiz,
      .pll_divcnt,
      .pll_apply,
      .pll_done,
      .pll_locked,
      .pll_error);

   // Scanout enable out to the fetch master: when software clears CTRL.VEN the
   // master stops issuing reads, so its in-flight reads can drain before the
   // framebuffer mapping is torn down.
   assign ven = ctrl.ven;

   // Color Look Up Table (CLUT)
   vctrl_clut u_clut
     (.clk_sys,
      .rst_sys,
      .cfg_clut_req,
      .cfg_clut_adr (cfg_adr[ 9:2]),
      .cfg_clut_we  (cfg_we),
      .cfg_clut_d   (cfg_d  [23:0]),
      .cfg_clut_ack,
      .cfg_clut_q,
      .cp_clut_req,
      .cp_clut_adr,
      .cp_clut_q,
      .cp_clut_ack);

   // VEN crosses into the pixel domain as a frame-aligned scan ENABLE (not a
   // reset). The pixel-domain reset (rst_pix) is board-generated from the same
   // source as rst_sys -- so disabling video no longer async-resets the pixel
   // domain (which used to tear the line-buffer/frame_sys CDCs and strand the
   // fetch path). See vctrl_timing.
   synchronizer #(2) u_ven_pix_sync
     (.clk(clk_pix), .d(ctrl.ven), .q(ven_pix));

   // Video Timing module
   vctrl_timing u_timing
     (.clk_pix,
      .rst_pix,
      .ven_pix,
      .htim,
      .vtim,
      .hspol    (ctrl.hsl),
      .vspol    (ctrl.vsl),
      .is_dbl   (ctrl.dbl),
      .hsync,
      .vsync,
      .frame,
      .line,
      .gate);

   // Frame buffer logic
   vctrl_fbuff #
     (.VR_SIZE  (VR_SIZE),
      .VR_DATAW (VR_DATAW),
      .LB_DEPTH (LB_DEPTH))
   u_fbuff
     (.clk_sys,
      .rst_sys,
      .fb_rdreq,
      .fb_raddr,
      .fb_rdack,
      .fb_rdata,
      .fb_rvalid,
      .frame_sys,
      .pitch,
      .thgate   (htim.tgate),
      .is_dbl   (ctrl.dbl),
      .bpp      (ctrl.cd),
      .is_pc    (ctrl.pc),
      .clut_req (cp_clut_req),
      .clut_offs(cp_clut_adr),
      .clut_q   (cp_clut_q),
      .clut_ack (cp_clut_ack),
      .clk_pix,
      .rst_pix,
      .hsync,
      .vsync,
      .frame,
      .line,
      .gate,
      .vga_r,
      .vga_g,
      .vga_b,
      .vga_bl,
      .vga_hs,
      .vga_vs);

endmodule // vctrl_core
