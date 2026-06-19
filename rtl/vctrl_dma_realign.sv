// -*- mode: verilog; mode: font-lock; indent-tabs-mode: nil -*-
// vi: set et ts=3 sw=3 sts=3:
//
// Copyright 2026 Steffen Persvold
// SPDX-License-Identifier: Apache-2.0
//
// ========================================================================
// File        : vctrl_dma_realign.sv
// Author      : Steffen Persvold
// Created     : June 19, 2026
// ========================================================================
// Description : Source-stream byte realigner for unaligned 2D blend/blit.
//
//   The read/write masters round every transfer base down to a beat and the
//   blend ALU composites source beat i lane p over destination beat i lane p
//   column-for-column. So a BLEND is only correct when the source and
//   destination share the same sub-beat (byte) offset. To compose a sprite
//   (e.g. a cursor) onto an arbitrary destination X, this block re-chunks the
//   beat-aligned source read stream into a stream aligned to the DESTINATION's
//   sub-beat offset, so source byte k lands in the lane the destination byte k
//   occupies. The destination read-back and write stay on the destination's
//   natural beat grid; the write master masks the partial first/last beats, so
//   bytes outside [dst, dst+width) (which this block fills with don't-care) are
//   blended but never written.
//
//   It is a byte accumulator, not a fixed window, because the source and
//   destination spans can differ by one beat: per row it consumes @in_beats
//   source beats and emits exactly @out_beats aligned beats. delta = src_off -
//   dst_off (bytes); a positive delta drops `delta` leading source bytes, a
//   negative delta prepends `-delta` don't-care bytes -- after that the byte
//   stream is simply re-cut into 32-byte beats. shift 0 (src_off==dst_off, e.g.
//   every COPY/FILL) is a pass-through.
// ========================================================================
//

module vctrl_dma_realign
  #(
    parameter  integer AXI_DATA_WIDTH = 256,
    localparam integer STRB_WIDTH     = AXI_DATA_WIDTH/8,        // bytes per beat
    localparam integer OFFW           = $clog2(STRB_WIDTH),      // sub-beat offset bits
    localparam integer ACCB           = 2*STRB_WIDTH,            // accumulator bytes
    localparam integer ACC_NW         = $clog2(ACCB+1)
    )
   (
    input  logic                       clk,
    input  logic                       rst,

    // ----------------------------------------------------------------------
    // Per-row setup, latched on @start (one cycle, at the source-row launch)
    // ----------------------------------------------------------------------
    input  logic                       start,
    input  logic [OFFW-1:0]            src_off,    // src_row_addr & (STRB_WIDTH-1)
    input  logic [OFFW-1:0]            dst_off,    // dst_row_addr & (STRB_WIDTH-1)
    input  logic [31:0]                out_beats,  // dst row beats to emit (>=1)

    // ----------------------------------------------------------------------
    // Beat-aligned source read stream in  ->  destination-aligned stream out
    // ----------------------------------------------------------------------
    input  logic [31:0]                in_beats,   // src row beats that will arrive
    input  logic                       in_valid,
    input  logic [AXI_DATA_WIDTH-1:0]  in_data,
    output logic                       out_valid,
    output logic [AXI_DATA_WIDTH-1:0]  out_data,

    // High from @start until all @out_beats have been emitted (drains the tail
    // flush after the last source beat); the parent gates the row-done edge on
    // ~busy so the realigned tail beat lands in the FIFO first.
    output logic                       busy
    );

   logic [ACCB*8-1:0] acc;       // byte accumulator (little-endian: byte 0 = [7:0])
   logic [ACC_NW-1:0] acc_n;     // valid bytes in acc
   logic [OFFW-1:0]   skip;      // remaining leading source bytes to drop
   logic [31:0]       emit_cnt;  // beats emitted this row
   logic [31:0]       in_cnt;    // source beats consumed this row
   logic [31:0]       want;      // out_beats latched
   logic [31:0]       nin;       // in_beats latched
   logic              run;       // row in progress

   // delta = src_off - dst_off (signed). delta>0 => drop `delta` leading source
   // bytes; delta<0 => prepend `-delta` don't-care bytes. Sampled at @start.
   wire signed [OFFW:0] delta = $signed({1'b0, src_off}) - $signed({1'b0, dst_off});
   wire [OFFW-1:0]      skip0 = (delta > 0) ? OFFW'(delta)    : '0;
   wire [ACC_NW-1:0]    pad0  = (delta < 0) ? ACC_NW'(-delta) : '0;

   // All source beats consumed: flush the remaining output beats from the
   // buffered bytes (their high lanes are tail, masked off by the writer).
   wire drained  = (in_cnt >= nin);
   // emit a beat when a full one is buffered, or when draining the tail
   wire can_emit = run & (emit_cnt < want) &
                   ((acc_n >= ACC_NW'(STRB_WIDTH)) | drained);
   // consume a source beat while the row still expects input
   wire take_in  = run & in_valid & (in_cnt < nin);

   // source bytes contributed by this input beat after dropping `skip` leading
   wire [AXI_DATA_WIDTH-1:0] in_drop = in_data >> (skip * 8);
   wire [ACC_NW-1:0]         in_len  = ACC_NW'(STRB_WIDTH) - ACC_NW'(skip);

   assign out_valid = can_emit;
   assign out_data  = acc[AXI_DATA_WIDTH-1:0];
   assign busy      = run;

   // accumulator state after an (optional) emit this cycle (a tail-flush beat
   // may drain fewer than a full beat of valid bytes -> clamp to zero)
   wire [ACC_NW-1:0]  acc_drop = (acc_n >= ACC_NW'(STRB_WIDTH)) ? ACC_NW'(STRB_WIDTH) : acc_n;
   wire [ACCB*8-1:0]  acc_e    = can_emit ? (acc >> AXI_DATA_WIDTH) : acc;
   wire [ACC_NW-1:0]  acc_n_e  = can_emit ? (acc_n - acc_drop)      : acc_n;

   always_ff @(posedge clk) begin
      if (start) begin
         // delta>0 => skip leading source bytes; delta<0 => prepend don't-care.
         acc      <= '0;
         acc_n    <= pad0;
         skip     <= skip0;
         emit_cnt <= '0;
         in_cnt   <= '0;
         want     <= out_beats;
         nin      <= in_beats;
         run      <= 1'b1;
      end
      else if (run) begin
         emit_cnt <= emit_cnt + (can_emit ? 32'd1 : 32'd0);
         in_cnt   <= in_cnt   + (take_in  ? 32'd1 : 32'd0);

         if (take_in) begin
            // append this beat's (post-skip) bytes at the post-emit fill point
            acc   <= acc_e | ({{(ACCB*8-AXI_DATA_WIDTH){1'b0}}, in_drop} << (acc_n_e * 8));
            acc_n <= acc_n_e + in_len;
            skip  <= '0;
         end
         else begin
            acc   <= acc_e;
            acc_n <= acc_n_e;
         end

         // row finishes once the last wanted beat has been emitted this cycle
         if (can_emit & (emit_cnt == want - 1))
           run <= 1'b0;
      end

      if (rst) begin
         run      <= 1'b0;
         acc_n    <= '0;
         skip     <= '0;
         emit_cnt <= '0;
         in_cnt   <= '0;
      end
   end

endmodule // vctrl_dma_realign
