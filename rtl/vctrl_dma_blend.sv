// -*- mode: verilog; mode: font-lock; indent-tabs-mode: nil -*-
// vi: set et ts=3 sw=3 sts=3:
//
// Copyright 2026 Steffen Persvold
// SPDX-License-Identifier: Apache-2.0
//
// ========================================================================
// File        : vctrl_dma_blend.sv
// Author      : Steffen Persvold
// Created     : June 18, 2026
// ========================================================================
// Description : Pipelined per-beat SRC_OVER compositor (premultiplied alpha).
//
//   Treats each beat as AXI_DATA_WIDTH/32 packed ARGB8888 pixels (alpha =
//   byte [31:24]) and computes, per channel:
//
//       out = src + div255(dst * (255 - src_alpha))
//
//   i.e. Porter-Duff "source over destination" with premultiplied operands
//   (the Wayland/DRM convention) -- no src*alpha multiply needed. All four
//   bytes (A,R,G,B) use the same expression, so the RGB byte order is
//   irrelevant as long as alpha occupies [31:24].
//
//   div255(x) = (t + (t >> 8)) >> 8, t = x + 128 -- the standard rounding
//   approximation, exact over x in [0, 255*255].
//
//   LATENCY-stage pipeline (one operation per stage), no backpressure (the
//   caller paces `in_valid` to its output FIFO; `o` is valid LATENCY cycles
//   later, tracked by `out_valid`):
//
//     1  input register   -- isolates the upstream (FIFO/memory) read register
//                            from the multiply so they are not chained
//     2  operand register  -- ia = 255 - src_alpha; multiply operands
//     3  product register  -- dst * ia
//     4  div255
//     5  src + div255(...), saturated
//
//   Stages 2-3 register the multiply's operands and product, which lets the
//   synthesizer pipeline it with no target-specific primitive: a hard-
//   multiplier flow absorbs the registers into the multiplier's own stages,
//   a standard-cell flow retimes the multiplier logic across them. Stage 1
//   keeps that absorbed input register off the source-memory read path.
// ========================================================================
//

module vctrl_dma_blend
  #(
    parameter  integer AXI_DATA_WIDTH = 256,
    localparam integer NPIX           = AXI_DATA_WIDTH/32
    )
   (
    input  logic                      clk,
    input  logic                      rst,
    input  logic                      in_valid,   // s/d present this cycle
    input  logic [AXI_DATA_WIDTH-1:0] s,          // source beat      (premult ARGB8888 x NPIX)
    input  logic [AXI_DATA_WIDTH-1:0] d,          // destination beat (premult ARGB8888 x NPIX)
    output logic                      out_valid,  // o valid (LATENCY cycles after in_valid)
    output logic [AXI_DATA_WIDTH-1:0] o           // s OVER d
    );

   localparam int LATENCY = 5;

   function automatic logic [7:0] div255(input logic [15:0] x);
      logic [16:0] t;
      t      = 17'(x) + 17'd128;
      div255 = 8'((t + (t >> 8)) >> 8);
   endfunction

   function automatic logic [7:0] satadd(input logic [7:0] a, input logic [7:0] b);
      logic [8:0] s9;
      s9     = 9'(a) + 9'(b);
      satadd = s9[8] ? 8'hFF : s9[7:0];
   endfunction

   // stage 1: register the raw input beats so the source read register does
   // not feed the multiply input register directly (breaks the memory->mult path)
   logic [AXI_DATA_WIDTH-1:0] s1_s, s1_d;
   always_ff @(posedge clk) begin
      s1_s <= s;
      s1_d <= d;
   end

   genvar p;
   generate
      for (p = 0; p < NPIX; p++) begin : g_px
         // channel bytes: [0]=A(31:24) [1]=R(23:16) [2]=G(15:8) [3]=B(7:0)
         logic  [7:0] s2_sc [4], s2_dc [4], s2_ia;   // stage 2: operands, ia=255-srcA
         logic [15:0] s3_prod [4];                   // stage 3: dst*ia product
         logic  [7:0] s3_sc [4];
         logic  [7:0] s4_q [4], s4_sc [4];           // stage 4: div255 result
         logic  [7:0] s5_o [4];                      // stage 5: src + div255(...), sat

         always_ff @(posedge clk) begin
            // stage 2: operands from the registered beats; ia = 255 - source alpha
            s2_ia    <= 8'd255 - s1_s[p*32+24 +: 8];
            s2_sc[0] <= s1_s[p*32+24 +: 8];  s2_dc[0] <= s1_d[p*32+24 +: 8];
            s2_sc[1] <= s1_s[p*32+16 +: 8];  s2_dc[1] <= s1_d[p*32+16 +: 8];
            s2_sc[2] <= s1_s[p*32+ 8 +: 8];  s2_dc[2] <= s1_d[p*32+ 8 +: 8];
            s2_sc[3] <= s1_s[p*32+ 0 +: 8];  s2_dc[3] <= s1_d[p*32+ 0 +: 8];

            // stage 3: multiply dst*ia (registered operands + product), carry src
            for (int k = 0; k < 4; k++) begin
               s3_prod[k] <= s2_dc[k] * s2_ia;
               s3_sc[k]   <= s2_sc[k];
            end

            // stage 4: div255, carry src
            for (int k = 0; k < 4; k++) begin
               s4_q[k]  <= div255(s3_prod[k]);
               s4_sc[k] <= s3_sc[k];
            end

            // stage 5: src + div255(dst*ia), saturated
            for (int k = 0; k < 4; k++)
              s5_o[k] <= satadd(s4_sc[k], s4_q[k]);
         end

         assign o[p*32+24 +: 8] = s5_o[0];
         assign o[p*32+16 +: 8] = s5_o[1];
         assign o[p*32+ 8 +: 8] = s5_o[2];
         assign o[p*32+ 0 +: 8] = s5_o[3];
      end
   endgenerate

   // valid follows the data down the same LATENCY stages
   logic [LATENCY-1:0] vld;
   always_ff @(posedge clk) begin
      vld <= {vld[LATENCY-2:0], in_valid};
      if (rst)
        vld <= '0;
   end
   assign out_valid = vld[LATENCY-1];

endmodule // vctrl_dma_blend
