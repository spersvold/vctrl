// -*- mode: verilog; mode: font-lock; indent-tabs-mode: nil -*-
// vi: set et ts=3 sw=3 sts=3:
//
// Copyright 2021-2026 Steffen Persvold
// SPDX-License-Identifier: Apache-2.0
//
// ========================================================================
// File        : vctrl_lbuff.sv
// Author      : Steffen Persvold
// Created     : April 15, 2021
// ========================================================================
// Description : Video Controller Line buffer (asynchronous)
// ========================================================================
//

module vctrl_lbuff import vctrl_pkg::*;
  #(
    parameter LB_DEPTH = 1024           // Max entries in line buffer
    )
   (
    input  logic            clk_in,     // input clock
    input  logic            clk_out,    // output clock
    input  logic            rst_out,    // output reset
    input  logic [11:0]     thgate,     // horizontal width (-1)
    input  logic            is_dbl,     // in doublescan mode
    output logic            data_req,   // request input data (clk_in)
    input  logic            en_in,      // enable input (clk_in)
    input  lb_data_t        din,        // data in (clk_in)
    input  logic            frame,      // start a new frame (clk_out)
    input  logic            line,       // start a new line (clk_out)
    input  logic            en_out,     // enable output (clk_out)
    output lb_data_t        dout        // data out (clk_out)
    );

   // ========================================================================
   // Internal Nets
   // ========================================================================

   typedef logic [$clog2(LB_DEPTH)-1:0] lb_addr_t;

   lb_addr_t                addr_in,   // input address (write)
                            addr_out;  // output address (pixel counter)

   logic                    get_data,  // fresh data needed
                            cnt_v,
                            cnt_h,     // scale counters
                            set_end;   // line set end

   // ========================================================================
   // ========================================================================

   // read data in
   always_ff @(posedge clk_in) begin
      if (en_in) addr_in <= addr_in + 1'b1;
      if (data_req) addr_in <= '0;  // reset addr_in when we request new data
   end

   // request new data on at end of line set (needs to be in clk_in domain)
   cdc_tgl u_xd_req (.clk_i(clk_out), .rst_i(rst_out), .clk_o(clk_in), .i(get_data), .o(data_req));

   // when scan doubling is enabled, we output pixels at half the speed
   always_ff @(posedge clk_out)
     if (rst_out | frame) cnt_h <= ~is_dbl;
     else                 cnt_h <= (is_dbl) ? ~cnt_h : cnt_h;

   // scan doubler logic, when enabled draws every line twice
   always_ff @(posedge clk_out)
     if (rst_out | frame) begin  // reset addr and counters at frame start
        cnt_v <= ~is_dbl;
        set_end <= is_dbl;  // ensure first line of frame triggers data_req
        addr_out <= '0;
     end
     else if (en_out & ~set_end & cnt_h) begin
        if (addr_out == lb_addr_t'(thgate)) begin  // end of line
           addr_out <= '0;
           if (cnt_v) begin  // end of line set
              cnt_v <= ~is_dbl;
              set_end <= is_dbl;
           end
           else cnt_v <= ~cnt_v;
        end
        else addr_out <= addr_out + 1'b1;
     end
     else if (get_data) set_end <= 1'b0;

   assign get_data = line & (set_end | ~is_dbl);

   // Linebuffer memory
   bram_sdp #
     (.WIDTH ($bits(din)),
      .DEPTH (LB_DEPTH))
   u_mem
     (.clk_write    (clk_in),
      .clk_read     (clk_out),
      .we           (en_in),
      .re           (1'b1),
      .addr_write   (addr_in),
      .addr_read    (addr_out),
      .data_in      (din),
      .data_out     (dout));

endmodule // linebuffer
