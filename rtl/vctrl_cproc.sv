// -*- mode: verilog; mode: font-lock; indent-tabs-mode: nil -*-
// vi: set et ts=3 sw=3 sts=3:
//
// Copyright 2021-2026 Steffen Persvold
// SPDX-License-Identifier: Apache-2.0
//
// ========================================================================
// File        : vctrl_cproc.sv
// Author      : Steffen Persvold
// Created     : April 15, 2021
// ========================================================================
// Description : Video Controller Color processing unit
// ========================================================================
//

module vctrl_cproc import vctrl_pkg::*;
  #(
    parameter  integer VR_SIZE  = 640 * 480 * 2, // Size of Frame Buffer memory (bytes)
    parameter  integer VR_DATAW = 32,            // Frame Buffer Data Width (bits)
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

    input  logic                   frame_sys,    // Start of new frame (in system clock domain)

    input  logic [           11:0] pitch,        // Horizontal line pitch
    input  logic [           11:0] thgate,       // Horizontal width
    input  bpp_t                   bpp,          // bits per pixel
    input  logic                   is_pc,        // pseudocolor mode

    output logic                   clut_req,     // clut request
    output logic [            7:0] clut_offs,    // clut offset
    input  logic [           23:0] clut_q,       // clut data in
    input  logic                   clut_ack,     // clut acknowledge

    input  logic                   lb_data_req,  // LB requesting data
    output lb_data_t               lb_din,       // data to linebuffer
    output logic                   lb_en_in
    );

   // ========================================================================
   // Internal Nets
   // ========================================================================
   localparam VR_IDX   = $clog2(VR_DATAW/8);

   typedef logic [$bits(thgate)-1:0] cnt_t;
   cnt_t                           cnt_h;        // Horizontal counter

   typedef logic [VR_IDX  -1:0] vr_idx_t;
   vr_idx_t                        idx,          // Byte index
                                   idx_next;

   typedef logic [VR_ADDRW-1:0] vr_adr_t;
   vr_adr_t                        fb_raddr_next;

   // ========================================================================
   // ========================================================================

   // Load data from FB into LB
   always_ff @(posedge clk_sys)
     begin
        if (fb_rdreq & fb_rdack) begin  // advance to start of next line
           cnt_h <= cnt_h + cnt_t'(1);
           fb_raddr <= fb_raddr_next;
           if (cnt_h == thgate)
             fb_rdreq <= '0; // last request
        end

        if (fb_rvalid) begin // increment mux index when receiving valid data
           idx <= idx_next;
        end

        if (lb_data_req) begin // single pulse
           idx <= '0; // start on first byte received
           cnt_h <= '0;  // start new line
           fb_rdreq <= '1; // request frame buffer read
           if (|fb_raddr) // for every line after the first we add pitch
             fb_raddr <= fb_raddr + vr_adr_t'(pitch);
        end

        if (frame_sys) begin
           fb_raddr <= '0; // new frame
           cnt_h <= '0;
        end

        if (rst_sys) begin
           idx <= '0;
           cnt_h <= '0;
           fb_raddr <= '0;
           fb_rdreq <= '0;
        end
     end

   // Increment Frame Buffer read address and index based on BitsPerPixel
   assign fb_raddr_next = fb_raddr + vr_adr_t'(fb_cidxb(bpp));
   assign idx_next = idx + vr_idx_t'(fb_cidxb(bpp));

   // Shift logic required for 24bpp
   logic [VR_DATAW -1:0] fb_rdata_d1;
   always_ff @(posedge clk_sys)
     if (fb_rvalid)
       fb_rdata_d1 <= fb_rdata;

   logic                 fb_rvalid_d1;
   always_ff @(posedge clk_sys)
     fb_rvalid_d1 <= fb_rvalid;

   vr_idx_t              idx_d1;
   always_ff @(posedge clk_sys)
     idx_d1 <= idx;

   assign clut_req = is_pc & (bpp == BPP_8) & fb_rvalid;

   // Convert Frame Buffer data to truecolor RGB888
   generate if (VR_DATAW == 32) begin: dataw_32
      always_ff @(posedge clk_sys)
        unique casez ({is_pc,bpp})
          {1'b?, BPP_32}: begin:rgb_0888
             lb_din.red   <= fb_rdata[23:16];
             lb_din.green <= fb_rdata[15: 8];
             lb_din.blue  <= fb_rdata[ 7: 0];
             lb_en_in     <= fb_rvalid;
          end
          {1'b?, BPP_24}: begin:rgb_888
             unique case (idx_d1)
               2'd3: begin
                  lb_din.red   <= fb_rdata   [15: 8];
                  lb_din.green <= fb_rdata   [ 7: 0];
                  lb_din.blue  <= fb_rdata_d1[31:24];
               end
               2'd2: begin
                  lb_din.red   <= fb_rdata   [ 7: 0];
                  lb_din.green <= fb_rdata_d1[31:24];
                  lb_din.blue  <= fb_rdata_d1[23:16];
               end
               2'd1: begin
                  lb_din.red   <= fb_rdata_d1[31:24];
                  lb_din.green <= fb_rdata_d1[23:16];
                  lb_din.blue  <= fb_rdata_d1[15: 8];
               end
               default: begin
                  lb_din.red   <= fb_rdata_d1[23:16];
                  lb_din.green <= fb_rdata_d1[15: 8];
                  lb_din.blue  <= fb_rdata_d1[ 7: 0];
               end
             endcase
             lb_en_in     <= fb_rvalid_d1;
          end
          {1'b?, BPP_16}: begin:rgb_565
             unique case (idx[1])
               1'b1: begin
                  lb_din.red   <= {fb_rdata[31:27], 3'b000};
                  lb_din.green <= {fb_rdata[26:21], 2'b00 };
                  lb_din.blue  <= {fb_rdata[20:16], 3'b000};
               end
               default: begin
                  lb_din.red   <= {fb_rdata[15:11], 3'b000};
                  lb_din.green <= {fb_rdata[10: 5], 2'b00 };
                  lb_din.blue  <= {fb_rdata[ 4: 0], 3'b000};
               end
             endcase
             lb_en_in     <= fb_rvalid;
          end
          {1'b1, BPP_8}: begin:rgb_lut8
             lb_din.red   <= clut_q[23:16];
             lb_din.green <= clut_q[15: 8];
             lb_din.blue  <= clut_q[ 7: 0];
             lb_en_in     <= clut_ack;
          end
          default: begin:bw
             unique case (idx)
               2'd3: begin
                  lb_din.red   <= fb_rdata[31:24];
                  lb_din.green <= fb_rdata[31:24];
                  lb_din.blue  <= fb_rdata[31:24];
               end
               2'd2: begin
                  lb_din.red   <= fb_rdata[23:16];
                  lb_din.green <= fb_rdata[23:16];
                  lb_din.blue  <= fb_rdata[23:16];
               end
               2'd1: begin
                  lb_din.red   <= fb_rdata[15: 8];
                  lb_din.green <= fb_rdata[15: 8];
                  lb_din.blue  <= fb_rdata[15: 8];
               end
               default: begin
                  lb_din.red   <= fb_rdata[ 7: 0];
                  lb_din.green <= fb_rdata[ 7: 0];
                  lb_din.blue  <= fb_rdata[ 7: 0];
               end
             endcase
             lb_en_in     <= fb_rvalid;
          end
        endcase

      always_comb
        unique case (idx)
          2'd3   : clut_offs = fb_rdata[31:24];
          2'd2   : clut_offs = fb_rdata[23:16];
          2'd1   : clut_offs = fb_rdata[15: 8];
          default: clut_offs = fb_rdata[ 7: 0];
        endcase

   end // block: dataw_32
   else begin: err
      $fatal("ERROR: Unsupported Frame Buffer Data Width %d", VR_DATAW);
   end endgenerate

endmodule // vctrl_cproc
