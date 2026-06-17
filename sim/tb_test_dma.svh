// -*- mode: verilog; mode: font-lock; indent-tabs-mode: nil -*-
// vi: set et ts=3 sw=3 sts=3:
//
// Copyright 2026 Steffen Persvold
// SPDX-License-Identifier: Apache-2.0
//
// ========================================================================
// File        : tb_test_dma.svh
// Author      : Steffen Persvold
// Created     : June 17, 2026
// ========================================================================
// Description : DMA engine testcase. Owns the cmd_* register-bus BFM and
//               drives a linear (height 1) host->dest copy: prefill the
//               source memory, program a COPY2D descriptor, ring the
//               doorbell, wait for the completion interrupt, then verify
//               the destination beat-for-beat. Selected with +TESTCASE=dma.
// ========================================================================
//

   // cmd_* register byte offsets (see vctrl_pkg DMA_*_ADR word addresses)
   localparam DMA_CTRL_OFF     = 'h04;
   localparam DMA_IRQ_OFF      = 'h0C;
   localparam DMA_IRQEN_OFF    = 'h10;
   localparam DMA_SRC_OFF      = 'h20;
   localparam DMA_DST_OFF      = 'h24;
   localparam DMA_SRCPITCH_OFF = 'h28;
   localparam DMA_DSTPITCH_OFF = 'h2C;
   localparam DMA_WIDTH_OFF    = 'h30;
   localparam DMA_HEIGHT_OFF   = 'h34;
   localparam DMA_OP_OFF       = 'h38;
   localparam DMA_DOORBELL_OFF = 'h3C;
   localparam DMA_FENCE_OFF    = 'h40;
   localparam DMA_RINGBASE_OFF = 'h50;
   localparam DMA_RINGSIZE_OFF = 'h54;
   localparam DMA_RINGTAIL_OFF = 'h58;
   localparam DMA_RINGHEAD_OFF = 'h5C;

   // copy parameters: 100 beats exercises 6 full bursts + a short final burst
   localparam int          DMA_TEST_NBEATS = 100;
   localparam logic [31:0] DMA_TEST_SRC    = 32'h0000_0000;
   localparam logic [31:0] DMA_TEST_DST    = 32'h0000_1000;
   localparam logic [31:0] DMA_TEST_SEQNO  = 32'hABCD_0001;

   // ----------------------------------------------------------------------
   // cmd_* register-bus BFM. cmd_req is held for exactly one cycle per
   // access (the slave write executes once -- a held req would re-issue,
   // and a doorbell must fire its desc_go pulse only once).
   // ----------------------------------------------------------------------
   task automatic cmd_write(input logic [11:0] off, input logic [31:0] data);
      begin
         @(posedge clk_sys); #0.1ns;
         cmd_req = 1'b1; cmd_we = 1'b1; cmd_be = 4'hf;
         cmd_adr = off[11:2]; cmd_d = data;
         @(posedge clk_sys); #0.1ns;
         cmd_req = 1'b0; cmd_we = 1'b0; cmd_adr = '0; cmd_d = '0;
      end
   endtask // cmd_write

   task automatic cmd_read(input logic [11:0] off, output logic [31:0] data);
      begin
         @(posedge clk_sys); #0.1ns;
         cmd_req = 1'b1; cmd_we = 1'b0; cmd_be = 4'hf; cmd_adr = off[11:2];
         @(posedge clk_sys); #0.1ns;
         cmd_req = 1'b0; cmd_adr = '0;
         @(posedge clk_sys); #0.1ns;
         data = cmd_q;
      end
   endtask // cmd_read

   // ----------------------------------------------------------------------
   task test_dma;

      logic [31:0] rdata;
      int          errors;
      int          si, di;
      realtime     t0, t1;
      int          cycles;

      begin
         // prefill the source with a recognizable per-beat pattern; clear the
         // destination window so a missed write would show up as a mismatch
         for (int i = 0; i < DMA_TEST_NBEATS; i++) begin
            u_src.mem[(DMA_TEST_SRC >> 5) + i] = {8{32'hC0DE_0000 + 32'(i)}};
            u_dst.mem[(DMA_TEST_DST >> 5) + i] = '0;
         end

         // enable the engine and the completion interrupt
         cmd_write(DMA_IRQEN_OFF, 32'h1);                 // IRQEN.done
         cmd_write(DMA_CTRL_OFF,  32'h1);                 // CTRL.enable

         // program a linear COPY2D descriptor (height 1)
         cmd_write(DMA_SRC_OFF,      DMA_TEST_SRC);
         cmd_write(DMA_DST_OFF,      DMA_TEST_DST);
         cmd_write(DMA_SRCPITCH_OFF, DMA_TEST_NBEATS * 32);
         cmd_write(DMA_DSTPITCH_OFF, DMA_TEST_NBEATS * 32);
         cmd_write(DMA_WIDTH_OFF,    DMA_TEST_NBEATS * 32);
         cmd_write(DMA_HEIGHT_OFF,   32'h1);
         cmd_write(DMA_OP_OFF,       32'(DMA_OP_COPY2D));

         $display("%t INFO: DMA doorbell (seqno=%h, %0d beats)", $time, DMA_TEST_SEQNO, DMA_TEST_NBEATS);
         t0 = $realtime;
         cmd_write(DMA_DOORBELL_OFF, DMA_TEST_SEQNO);

         // wait for completion, clear the interrupt, read back the fence
         wait (dma_irq);
         t1 = $realtime;
         cycles = int'((t1 - t0) / CLK_SYS_PERIOD);
         cmd_write(DMA_IRQ_OFF, 32'h1);                   // W1C the done bit
         cmd_read (DMA_FENCE_OFF, rdata);
         if (rdata !== DMA_TEST_SEQNO)
           $display("%t ERROR: fence=%h expected %h", $time, rdata, DMA_TEST_SEQNO);

         // verify the destination beat-for-beat
         errors = 0;
         for (int i = 0; i < DMA_TEST_NBEATS; i++) begin
            si = (DMA_TEST_SRC >> 5) + i;
            di = (DMA_TEST_DST >> 5) + i;
            if (u_dst.mem[di] !== u_src.mem[si]) begin
               errors++;
               if (errors <= 4)
                 $display("  beat %0d MISMATCH dst=%h src=%h", i, u_dst.mem[di], u_src.mem[si]);
            end
         end

         if (errors == 0 && rdata === DMA_TEST_SEQNO) begin
            $display("%t INFO: DMA linear copy verified (%0d beats in %0d cycles, RESP_LATENCY=%0d)",
                     $time, DMA_TEST_NBEATS, cycles, DMA_RESP_LAT);
            result = 1'b1;
         end
         else
           $display("%t ERROR: DMA test failed (%0d mismatched beats)", $time, errors);
      end
   endtask // test_dma

   // ----------------------------------------------------------------------
   // Ring testcase: a ring of descriptors in source memory, published with a
   // single RING_TAIL doorbell; the engine fetches and executes each one.
   // ----------------------------------------------------------------------
   task test_dma_ring;

      localparam int          NENT     = 4;             // ring entries
      localparam int          RINGLOG  = 2;             // log2(NENT)
      localparam logic [31:0] RINGBASE = 32'h0000_0000; // ring (in source mem)
      localparam logic [31:0] SRCDATA  = 32'h0000_1000; // copy source (source mem)
      localparam logic [31:0] DSTDATA  = 32'h0000_2000; // copy dest (dest mem)
      localparam int          CBEATS   = 16;            // beats copied per entry
      localparam int          CBYTES   = CBEATS * 32;
      localparam logic [31:0] SEQ0     = 32'h0BAD_0000;

      dma_desc_t   d;
      logic [31:0] rdata;
      int          errors, si, di;

      begin
         // build the copy source data, clear the dest, and lay out the ring
         for (int e = 0; e < NENT; e++) begin
            for (int b = 0; b < CBEATS; b++) begin
               u_src.mem[(SRCDATA >> 5) + e*CBEATS + b] = {8{32'hD000_0000 + 32'(e*256 + b)}};
               u_dst.mem[(DSTDATA >> 5) + e*CBEATS + b] = '0;
            end
            d           = '0;
            d.opflags   = 32'(DMA_OP_COPY2D);
            d.seqno     = SEQ0 + 32'(e);
            d.src_addr  = SRCDATA + 32'(e*CBYTES);
            d.dst_addr  = DSTDATA + 32'(e*CBYTES);
            d.src_pitch = CBYTES;
            d.dst_pitch = CBYTES;
            d.width     = CBYTES;
            d.height    = 1;
            u_src.mem[(RINGBASE >> 5) + e] = d;   // one descriptor = one beat
         end

         // program the ring and enable ring mode
         cmd_write(DMA_IRQEN_OFF,    32'h1);
         cmd_write(DMA_RINGBASE_OFF, RINGBASE);
         cmd_write(DMA_RINGSIZE_OFF, RINGLOG);
         cmd_write(DMA_CTRL_OFF,     (32'h1 << DMA_CTRL_ENABLE) | (32'h1 << DMA_CTRL_RINGEN));

         // doorbell: publish all NENT entries (tail = NENT)
         $display("%t INFO: DMA ring doorbell (%0d entries)", $time, NENT);
         cmd_write(DMA_RINGTAIL_OFF, NENT);

         // wait until the engine has consumed the whole ring (head == tail)
         do cmd_read(DMA_RINGHEAD_OFF, rdata); while (rdata != NENT);

         // verify every entry's destination matches its source
         errors = 0;
         for (int e = 0; e < NENT; e++)
           for (int b = 0; b < CBEATS; b++) begin
              si = (SRCDATA >> 5) + e*CBEATS + b;
              di = (DSTDATA >> 5) + e*CBEATS + b;
              if (u_dst.mem[di] !== u_src.mem[si]) begin
                 errors++;
                 if (errors <= 4)
                   $display("  entry %0d beat %0d MISMATCH dst=%h src=%h",
                            e, b, u_dst.mem[di], u_src.mem[si]);
              end
           end

         cmd_read(DMA_FENCE_OFF, rdata);
         if (errors == 0 && rdata === (SEQ0 + 32'(NENT-1))) begin
            $display("%t INFO: DMA ring verified (%0d entries x %0d beats, fence=%h)",
                     $time, NENT, CBEATS, rdata);
            result = 1'b1;
         end
         else
           $display("%t ERROR: DMA ring failed (%0d mismatches, fence=%h)", $time, errors, rdata);
      end
   endtask // test_dma_ring

   // ----------------------------------------------------------------------
   // 2D testcase: copy a sub-rectangle (height>1) with mismatched source and
   // destination row pitches. Verifies the copied cells match the source AND
   // that the inter-row gap in the destination is untouched (proving the walk
   // copies `width` per row, not the whole pitch).
   // ----------------------------------------------------------------------
   task test_dma_2d;

      localparam int          WBEATS   = 4;              // rect width in beats (128 B)
      localparam int          HROWS    = 8;              // rect height in rows
      localparam int          SPITCH_B = 8 * 32;         // source row stride (256 B)
      localparam int          DPITCH_B = 6 * 32;         // dest row stride (192 B, != src)
      localparam int          WBYTES   = WBEATS * 32;
      localparam logic [31:0] SRC_FB   = 32'h0000_1000;  // source rect base (source mem)
      localparam logic [31:0] DST_FB   = 32'h0000_2000;  // dest rect base (dest mem)
      localparam logic [31:0] SEQNO    = 32'h2D00_0001;

      logic [31:0] rdata;
      int          errors, r, b, gi, si, di;

      begin
         // fill the source rect rows; clear the dest rect span (rows + gaps)
         for (r = 0; r < HROWS; r++) begin
            for (b = 0; b < WBEATS; b++)
              u_src.mem[(SRC_FB >> 5) + r*(SPITCH_B >> 5) + b] = {8{32'h2D00_0000 + 32'(r*16 + b)}};
            for (b = 0; b < (DPITCH_B >> 5); b++)
              u_dst.mem[(DST_FB >> 5) + r*(DPITCH_B >> 5) + b] = '0;
         end

         cmd_write(DMA_IRQEN_OFF,    32'h1);
         cmd_write(DMA_CTRL_OFF,     32'h1 << DMA_CTRL_ENABLE);
         cmd_write(DMA_SRC_OFF,      SRC_FB);
         cmd_write(DMA_DST_OFF,      DST_FB);
         cmd_write(DMA_SRCPITCH_OFF, SPITCH_B);
         cmd_write(DMA_DSTPITCH_OFF, DPITCH_B);
         cmd_write(DMA_WIDTH_OFF,    WBYTES);
         cmd_write(DMA_HEIGHT_OFF,   HROWS);
         cmd_write(DMA_OP_OFF,       32'(DMA_OP_COPY2D));

         $display("%t INFO: DMA 2D doorbell (%0d beats x %0d rows, spitch=%0d dpitch=%0d)",
                  $time, WBEATS, HROWS, SPITCH_B, DPITCH_B);
         cmd_write(DMA_DOORBELL_OFF, SEQNO);

         wait (dma_irq);
         cmd_write(DMA_IRQ_OFF, 32'h1);
         cmd_read (DMA_FENCE_OFF, rdata);

         errors = 0;
         for (r = 0; r < HROWS; r++) begin
            for (b = 0; b < WBEATS; b++) begin          // copied cells match source
               si = (SRC_FB >> 5) + r*(SPITCH_B >> 5) + b;
               di = (DST_FB >> 5) + r*(DPITCH_B >> 5) + b;
               if (u_dst.mem[di] !== u_src.mem[si]) begin
                  errors++;
                  if (errors <= 4)
                    $display("  row %0d beat %0d MISMATCH dst=%h src=%h", r, b, u_dst.mem[di], u_src.mem[si]);
               end
            end
            for (gi = WBEATS; gi < (DPITCH_B >> 5); gi++) begin  // gap untouched
               di = (DST_FB >> 5) + r*(DPITCH_B >> 5) + gi;
               if (u_dst.mem[di] !== '0) begin
                  errors++;
                  if (errors <= 8)
                    $display("  row %0d gap beat %0d OVERWRITTEN dst=%h", r, gi, u_dst.mem[di]);
               end
            end
         end

         if (errors == 0 && rdata === SEQNO) begin
            $display("%t INFO: DMA 2D copy verified (%0d rows x %0d beats, pitch %0d->%0d, fence=%h)",
                     $time, HROWS, WBEATS, SPITCH_B, DPITCH_B, rdata);
            result = 1'b1;
         end
         else
           $display("%t ERROR: DMA 2D failed (%0d mismatches, fence=%h)", $time, errors, rdata);
      end
   endtask // test_dma_2d

   // ----------------------------------------------------------------------
   // WSTRB testcase: a width that is not a beat multiple (3 full beats + a
   // 16-byte tail). Each row's tail beat must write only its low 16 bytes
   // (masked by wr_last_be); the high half and the inter-row gap stay zero.
   // ----------------------------------------------------------------------
   task test_dma_wstrb;

      localparam int          WFULL   = 3;                  // full beats per row
      localparam int          PBYTES  = 16;                 // partial tail bytes
      localparam int          PBITS   = PBYTES * 8;          // 128
      localparam int          WBYTES  = WFULL*32 + PBYTES;   // 112 (not 32-aligned)
      localparam int          HROWS   = 2;
      localparam int          PITCH_B = 6 * 32;              // 192 (gap after each row)
      localparam logic [31:0] SRC_FB  = 32'h0000_1000;
      localparam logic [31:0] DST_FB  = 32'h0000_2000;
      localparam logic [31:0] SEQNO   = 32'h57B0_0001;

      logic [31:0] rdata;
      int          errors, r, b, g, si, di;

      begin
         for (r = 0; r < HROWS; r++) begin
            for (b = 0; b <= WFULL; b++)               // WFULL full beats + 1 partial
              u_src.mem[(SRC_FB >> 5) + r*(PITCH_B >> 5) + b] = {8{32'h57B0_0000 + 32'(r*16 + b)}};
            for (b = 0; b < (PITCH_B >> 5); b++)
              u_dst.mem[(DST_FB >> 5) + r*(PITCH_B >> 5) + b] = '0;
         end

         cmd_write(DMA_IRQEN_OFF,    32'h1);
         cmd_write(DMA_CTRL_OFF,     32'h1 << DMA_CTRL_ENABLE);
         cmd_write(DMA_SRC_OFF,      SRC_FB);
         cmd_write(DMA_DST_OFF,      DST_FB);
         cmd_write(DMA_SRCPITCH_OFF, PITCH_B);
         cmd_write(DMA_DSTPITCH_OFF, PITCH_B);
         cmd_write(DMA_WIDTH_OFF,    WBYTES);
         cmd_write(DMA_HEIGHT_OFF,   HROWS);
         cmd_write(DMA_OP_OFF,       32'(DMA_OP_COPY2D));

         $display("%t INFO: DMA WSTRB doorbell (width=%0d B = %0d full + %0d tail, %0d rows)",
                  $time, WBYTES, WFULL, PBYTES, HROWS);
         cmd_write(DMA_DOORBELL_OFF, SEQNO);

         wait (dma_irq);
         cmd_write(DMA_IRQ_OFF, 32'h1);
         cmd_read (DMA_FENCE_OFF, rdata);

         errors = 0;
         for (r = 0; r < HROWS; r++) begin
            for (b = 0; b < WFULL; b++) begin              // full beats copy entirely
               si = (SRC_FB >> 5) + r*(PITCH_B >> 5) + b;
               di = (DST_FB >> 5) + r*(PITCH_B >> 5) + b;
               if (u_dst.mem[di] !== u_src.mem[si]) begin
                  errors++;
                  if (errors <= 4) $display("  row %0d full beat %0d MISMATCH", r, b);
               end
            end
            // partial tail beat: low PBYTES copied, high half untouched
            si = (SRC_FB >> 5) + r*(PITCH_B >> 5) + WFULL;
            di = (DST_FB >> 5) + r*(PITCH_B >> 5) + WFULL;
            if (u_dst.mem[di][PBITS-1:0] !== u_src.mem[si][PBITS-1:0]) begin
               errors++;
               $display("  row %0d tail-low MISMATCH dst=%h src=%h",
                        r, u_dst.mem[di][PBITS-1:0], u_src.mem[si][PBITS-1:0]);
            end
            if (u_dst.mem[di][AXI_DATA_WIDTH-1:PBITS] !== '0) begin
               errors++;
               $display("  row %0d tail-high OVERWRITTEN dst=%h", r, u_dst.mem[di][AXI_DATA_WIDTH-1:PBITS]);
            end
            // inter-row gap untouched
            for (g = WFULL+1; g < (PITCH_B >> 5); g++) begin
               di = (DST_FB >> 5) + r*(PITCH_B >> 5) + g;
               if (u_dst.mem[di] !== '0) begin
                  errors++;
                  if (errors <= 8) $display("  row %0d gap beat %0d OVERWRITTEN", r, g);
               end
            end
         end

         if (errors == 0 && rdata === SEQNO) begin
            $display("%t INFO: DMA WSTRB copy verified (%0d rows, %0d-byte tail masked, fence=%h)",
                     $time, HROWS, PBYTES, rdata);
            result = 1'b1;
         end
         else
           $display("%t ERROR: DMA WSTRB failed (%0d mismatches, fence=%h)", $time, errors, rdata);
      end
   endtask // test_dma_wstrb
