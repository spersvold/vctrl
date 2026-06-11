// -*- mode: verilog; mode: font-lock; indent-tabs-mode: nil -*-
// vi: set et ts=3 sw=3 sts=3:
//
// Copyright 2021-2026 Steffen Persvold
// SPDX-License-Identifier: Apache-2.0
//
// ========================================================================
// File        : vctrl_fbuff.sv
// Author      : Steffen Persvold
// Created     : April 15, 2021
// ========================================================================
// Description : Video Controller Frame buffer
// ========================================================================
//

module vctrl_fbuff import vctrl_pkg::*;
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

    output logic                   fb_rdreq,     // Memory read request
    output logic [VR_ADDRW   -1:0] fb_raddr,     // Memory read address
    input  logic                   fb_rdack,     // Memory read ackowledge
    input  logic [VR_DATAW   -1:0] fb_rdata,     // Memory read data
    input  logic                   fb_rvalid,    // Memory read data valid

    output logic                   frame_sys,    // Start of new frame (in system clock domain)

    input  logic [           11:0] pitch,        // Horizontal line pitch
    input  logic [           11:0] thgate,       // horizontal width (-1)
    input  logic                   is_dbl,       // In doublescan mode
    input  bpp_t                   bpp,          // bits per pixel
    input  logic                   is_pc,        // pseudocolor mode

    output logic                   clut_req,     // clut request
    output logic [            7:0] clut_offs,    // clut offset
    input  logic [           23:0] clut_q,       // clut data in
    input  logic                   clut_ack,     // clut acknowledge

    input  logic                   clk_pix,      // Pixel clock
    input  logic                   rst_pix,      // Video reset

    input  logic                   hsync,        // Horizontal sync
    input  logic                   vsync,        // Vertical sync
    input  logic                   frame,        // Start of frame
    input  logic                   line,         // Start of line
    input  logic                   gate,         // Data gate

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

   logic                           lb_data_req;  // LB requesting data
   lb_data_t                       lb_din;
   lb_data_t                       lb_dout;
   logic                           lb_en_in;

   // ========================================================================
   // ========================================================================

   cdc_tgl u_xd_frame (.clk_i(clk_pix), .rst_i(rst_pix), .clk_o(clk_sys), .i(frame), .o(frame_sys));

   vctrl_cproc #
     (.VR_SIZE  (VR_SIZE),
      .VR_DATAW (VR_DATAW))
   u_cproc
     (.clk_sys,
      .rst_sys,
      .fb_rdreq,
      .fb_raddr,
      .fb_rdack,
      .fb_rdata,
      .fb_rvalid,
      .frame_sys,
      .pitch,
      .thgate,
      .bpp,
      .is_pc,
      .clut_req,
      .clut_offs,
      .clut_q,
      .clut_ack,
      .lb_data_req,
      .lb_din,
      .lb_en_in);

   vctrl_lbuff #
     (.LB_DEPTH (LB_DEPTH))
   u_lbuff
     (.clk_in   (clk_sys),
      .clk_out  (clk_pix),
      .rst_out  (rst_pix),
      .thgate,
      .is_dbl,
      .data_req (lb_data_req),
      .en_in    (lb_en_in),
      .din      (lb_din),
      .frame,
      .line,
      .en_out   (gate),
      .dout     (lb_dout));

   // LB enable out: reading from LB RAM takes one cycle
   logic lb_en_out_p1;
   always_ff @(posedge clk_pix) lb_en_out_p1 <= gate; // spyglass disable ResetFlop-ML

   // Reading from LB takes one cycle: delay display signals to match
   logic hsync_p1, vsync_p1;
   always_ff @(posedge clk_pix) hsync_p1 <= hsync; // spyglass disable ResetFlop-ML
   always_ff @(posedge clk_pix) vsync_p1 <= vsync; // spyglass disable ResetFlop-ML

   // Video output, force to zero in blanking interval
   // spyglass disable_block ResetFlop-ML
   always_ff @(posedge clk_pix) begin
      vga_r  <= (lb_en_out_p1) ? lb_dout.red   : '0;
      vga_g  <= (lb_en_out_p1) ? lb_dout.green : '0;
      vga_b  <= (lb_en_out_p1) ? lb_dout.blue  : '0;
      vga_hs <= hsync_p1;
      vga_vs <= vsync_p1;
      vga_bl <= ~lb_en_out_p1;
   end
   // spyglass enable_block ResetFlop-ML

endmodule // vctrl_fbuff
