// -*- mode: verilog; mode: font-lock; indent-tabs-mode: nil -*-
// vi: set et ts=3 sw=3 sts=3:
//
// Copyright 2021-2026 Steffen Persvold
// SPDX-License-Identifier: Apache-2.0
//
// ========================================================================
// File        : vctrl_clut.sv
// Author      : Steffen Persvold
// Created     : April 15, 2021
// ========================================================================
// Description : Video Controller Color Look Up Table (CLUT)
// ========================================================================
//

module vctrl_clut
   (
    input  logic                   clk_sys,      // System clock
    input  logic                   rst_sys,      // System Reset Signal

    input  logic                   cfg_clut_req, // Config CLUT request
    input  logic [ 7:0]            cfg_clut_adr, // Config CLUT address
    input  logic                   cfg_clut_we,  // Config CLUT write enable
    input  logic [23:0]            cfg_clut_d,   // Config CLUT write data
    output logic                   cfg_clut_ack, // Config CLUT acknowledge
    output logic [23:0]            cfg_clut_q,   // Config CLUT read data

    input  logic                   cp_clut_req,  // Color processor CLUT request
    input  logic [ 7:0]            cp_clut_adr,  // Color processor CLUT address
    output logic [23:0]            cp_clut_q,    // Color processor CLUT read data
    output logic                   cp_clut_ack   // Color processor CLUT acknowledge
    );

   // ========================================================================
   // Local variable declarations
   // ========================================================================

   logic                           cfg_clut_req_dly,
                                   cfg_clut_req_ack;
   logic [ 7:0]                    cfg_clut_adr_dly;
   logic                           cfg_clut_we_dly;
   logic [23:0]                    cfg_clut_d_dly;

   logic                           cp_clut_req_dly,
                                   cp_clut_req_ack;

   logic                           mem_req,
                                   mem_gnt,
                                   mem_req_dly,
                                   mem_we;
   logic [ 7:0]                    mem_adr;
   logic [23:0]                    mem_d,
                                   mem_q;
   logic                           mem_ack;

   // ========================================================================
   // ========================================================================

   // Pipeline cfg_* signals
   always_ff @(posedge clk_sys)
     if (rst_sys) cfg_clut_req_dly <= 1'b0;
     else         cfg_clut_req_dly <= cfg_clut_req | (cfg_clut_req_dly & ~cfg_clut_req_ack);

   always_ff @(posedge clk_sys)
     if (cfg_clut_req)
       begin
          cfg_clut_adr_dly <= cfg_clut_adr;
          cfg_clut_we_dly  <= cfg_clut_we;
          cfg_clut_d_dly   <= cfg_clut_d;
       end

   // Framebuffer reads has priority
   assign cp_clut_req_ack  =  cp_clut_req & mem_gnt;
   assign cfg_clut_req_ack = ~cp_clut_req & cfg_clut_req_dly & mem_gnt;

   // Remember if FB requested the read (used for data demux later)
   always_ff @(posedge clk_sys)
     if      (rst_sys) cp_clut_req_dly <= 1'b0;
     else if (mem_req) cp_clut_req_dly <= cp_clut_req_ack;

   always_ff @(posedge clk_sys)
     if (rst_sys) mem_req_dly <= 1'b0;
     else         mem_req_dly <= mem_req | (mem_req_dly & ~mem_ack);

   assign mem_gnt = ~mem_req_dly | mem_ack;
   assign mem_req = (cfg_clut_req_dly | cp_clut_req) & mem_gnt;
   assign mem_we  = ~cp_clut_req & (cfg_clut_we_dly & cfg_clut_req_dly);
   assign mem_d   = cfg_clut_d_dly;

   always_comb
     unique case (cp_clut_req)
       1'b1   : mem_adr = cp_clut_adr;
       default: mem_adr = cfg_clut_adr_dly;
     endcase

   assign cfg_clut_q   = mem_q;
   assign cfg_clut_ack = mem_ack & ~cp_clut_req_dly;
   assign cp_clut_q    = mem_q;
   assign cp_clut_ack  = mem_ack &  cp_clut_req_dly;

   //
   // CLUT memory
   //
   bram_1rw #
     (.WIDTH ($bits(mem_d)),
      .DEPTH (2**$bits(mem_adr)))
   u_mem
     (.clk     (clk_sys),
      .we      (mem_we),
      .addr    (mem_adr),
      .data_in (mem_d),
      .data_out(mem_q));

   always_ff @(posedge clk_sys)
     if (rst_sys) mem_ack <= 1'b0;
     else         mem_ack <= mem_req;

endmodule // vctrl_clut
