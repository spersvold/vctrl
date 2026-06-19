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

   // ----------------------------------------------------------------------
   // FILL testcase: solid-fill a sub-rectangle (height>1) with a partial-width
   // tail and an inter-row gap. Every filled cell must equal the replicated
   // ARGB color; the masked tail-high half and the inter-row gap stay zero.
   // No source read is issued -- this exercises the write-only path.
   // ----------------------------------------------------------------------
   task test_dma_fill;

      localparam int          WFULL   = 3;                  // full beats per row
      localparam int          PBYTES  = 16;                 // partial tail bytes
      localparam int          PBITS   = PBYTES * 8;          // 128
      localparam int          WBYTES  = WFULL*32 + PBYTES;   // 112 (not 32-aligned)
      localparam int          HROWS   = 4;
      localparam int          PITCH_B = 6 * 32;              // 192 (gap after each row)
      localparam logic [31:0] DST_FB  = 32'h0000_2000;       // dest rect base (dest mem)
      localparam logic [31:0] COLOR   = 32'hFF20_A0FF;       // ARGB fill color
      localparam logic [31:0] SEQNO   = 32'hF111_0001;

      logic [AXI_DATA_WIDTH-1:0] beat;
      logic [31:0] rdata;
      int          errors, r, b, g, di;

      begin
         beat = {(AXI_DATA_WIDTH/32){COLOR}};

         // clear the dest rect span (rows + gaps) so any stray write shows up
         for (r = 0; r < HROWS; r++)
           for (b = 0; b < (PITCH_B >> 5); b++)
             u_dst.mem[(DST_FB >> 5) + r*(PITCH_B >> 5) + b] = '0;

         cmd_write(DMA_IRQEN_OFF,    32'h1);
         cmd_write(DMA_CTRL_OFF,     32'h1 << DMA_CTRL_ENABLE);
         cmd_write(DMA_SRC_OFF,      COLOR);             // FILL: src_addr = ARGB color
         cmd_write(DMA_DST_OFF,      DST_FB);
         cmd_write(DMA_SRCPITCH_OFF, 32'h0);             // unused by FILL
         cmd_write(DMA_DSTPITCH_OFF, PITCH_B);
         cmd_write(DMA_WIDTH_OFF,    WBYTES);
         cmd_write(DMA_HEIGHT_OFF,   HROWS);
         cmd_write(DMA_OP_OFF,       32'(DMA_OP_FILL));

         $display("%t INFO: DMA FILL doorbell (color=%h, width=%0d B = %0d full + %0d tail, %0d rows)",
                  $time, COLOR, WBYTES, WFULL, PBYTES, HROWS);
         cmd_write(DMA_DOORBELL_OFF, SEQNO);

         wait (dma_irq);
         cmd_write(DMA_IRQ_OFF, 32'h1);
         cmd_read (DMA_FENCE_OFF, rdata);

         errors = 0;
         for (r = 0; r < HROWS; r++) begin
            for (b = 0; b < WFULL; b++) begin              // full beats == replicated color
               di = (DST_FB >> 5) + r*(PITCH_B >> 5) + b;
               if (u_dst.mem[di] !== beat) begin
                  errors++;
                  if (errors <= 4) $display("  row %0d full beat %0d MISMATCH dst=%h", r, b, u_dst.mem[di]);
               end
            end
            // partial tail beat: low PBYTES filled, high half untouched
            di = (DST_FB >> 5) + r*(PITCH_B >> 5) + WFULL;
            if (u_dst.mem[di][PBITS-1:0] !== beat[PBITS-1:0]) begin
               errors++;
               $display("  row %0d tail-low MISMATCH dst=%h", r, u_dst.mem[di][PBITS-1:0]);
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
            $display("%t INFO: DMA FILL verified (%0d rows, color=%h, %0d-byte tail masked, fence=%h)",
                     $time, HROWS, COLOR, PBYTES, rdata);
            result = 1'b1;
         end
         else
           $display("%t ERROR: DMA FILL failed (%0d mismatches, fence=%h)", $time, errors, rdata);
      end
   endtask // test_dma_fill

   // ----------------------------------------------------------------------
   // 4 KiB-boundary testcase: a wide partial-width strided copy whose rows are
   // long enough -- and whose base is beat-aligned but NOT 512 B-aligned -- that
   // a max-length burst would cross a 4 KiB boundary. This is the X11 window-blit
   // geometry (and the one the contiguous, 4 KiB-aligned copy-on-flip path never
   // exercises). AXI4 forbids an INCR burst from crossing 4 KiB; the masters must
   // split there. The always-on checker in tb.sv latches axi4k_violation if any
   // issued burst crosses; this test fails on that flag. The data check is kept
   // too (a mis-split that dropped/duplicated beats would corrupt it) but cannot
   // by itself catch the crossing -- the axi_ram models linearize, returning
   // correct data even for an illegal burst, which is precisely why the bug
   // reached hardware.
   // ----------------------------------------------------------------------
   task test_dma_4k;

      localparam int          WBEATS  = 175;             // 5600 B/row (> 4 KiB)
      localparam int          HROWS   = 4;
      localparam int          PITCH_B = 7680;            // 1080p scanout pitch
      localparam int          WBYTES  = WBEATS * 32;
      localparam logic [31:0] SRC_FB  = 32'h0000_04C0;   // 1216 B: beat-aligned, not 512 B-aligned
      localparam logic [31:0] DST_FB  = 32'h0008_04C0;   // same low bits, clear of the source span
      localparam logic [31:0] SEQNO   = 32'h04C0_0001;

      logic [31:0] rdata;
      int          errors, r, b, si, di;

      begin
         axi4k_violation = 1'b0;

         // position-encoded source rows; clear the matching dest rows
         for (r = 0; r < HROWS; r++)
           for (b = 0; b < WBEATS; b++) begin
              u_src.mem[(SRC_FB >> 5) + r*(PITCH_B >> 5) + b] = {8{32'h4B00_0000 + 32'(r*256 + b)}};
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

         $display("%t INFO: DMA 4KiB-cross doorbell (%0d beats x %0d rows, base=%h pitch=%0d)",
                  $time, WBEATS, HROWS, SRC_FB, PITCH_B);
         cmd_write(DMA_DOORBELL_OFF, SEQNO);

         wait (dma_irq);
         cmd_write(DMA_IRQ_OFF, 32'h1);
         cmd_read (DMA_FENCE_OFF, rdata);

         errors = 0;
         for (r = 0; r < HROWS; r++)
           for (b = 0; b < WBEATS; b++) begin
              si = (SRC_FB >> 5) + r*(PITCH_B >> 5) + b;
              di = (DST_FB >> 5) + r*(PITCH_B >> 5) + b;
              if (u_dst.mem[di] !== u_src.mem[si]) begin
                 errors++;
                 if (errors <= 4)
                   $display("  row %0d beat %0d MISMATCH dst=%h src=%h", r, b, u_dst.mem[di], u_src.mem[si]);
              end
           end

         if (errors == 0 && rdata === SEQNO && !axi4k_violation) begin
            $display("%t INFO: DMA 4KiB-cross copy verified (%0d rows x %0d beats), no crossing burst",
                     $time, HROWS, WBEATS);
            result = 1'b1;
         end
         else
           $display("%t ERROR: DMA 4KiB-cross failed (errors=%0d fence=%h 4k_violation=%b)",
                    $time, errors, rdata, axi4k_violation);
      end
   endtask // test_dma_4k

   // ----------------------------------------------------------------------
   // Software reference for the premultiplied SRC_OVER compositor -- must
   // match vctrl_dma_blend exactly (same div255 rounding).
   // ----------------------------------------------------------------------
   function automatic logic [7:0] ref_div255(input logic [15:0] x);
      logic [16:0] t;
      t         = 17'(x) + 17'd128;
      ref_div255 = 8'((t + (t >> 8)) >> 8);
   endfunction
   function automatic logic [7:0] ref_chan(input logic [7:0] sc, input logic [7:0] dc, input logic [7:0] ia);
      logic [8:0] sum;
      sum      = 9'(sc) + 9'(ref_div255(16'(dc) * 16'(ia)));
      ref_chan = sum[8] ? 8'hFF : sum[7:0];
   endfunction
   function automatic logic [31:0] ref_over(input logic [31:0] s, input logic [31:0] d);
      logic [7:0] ia;
      ia = 8'd255 - s[31:24];
      ref_over = { ref_chan(s[31:24], d[31:24], ia),     // A
                   ref_chan(s[23:16], d[23:16], ia),     // R
                   ref_chan(s[15: 8], d[15: 8], ia),     // G
                   ref_chan(s[ 7: 0], d[ 7: 0], ia) };   // B
   endfunction

   // ----------------------------------------------------------------------
   // BLEND testcase: premultiplied SRC_OVER of a source row onto a destination
   // row, over a 2-row rect with a partial-width (non-beat-aligned) tail. The
   // engine reads the source row, reads the destination row, composites per
   // pixel, and writes the result back. In the bench both reads are served by
   // u_src (source over-layer and destination under-layer in separate regions)
   // and the result lands in u_dst; the same-address read-before-write ordering
   // is guaranteed structurally (each write beat is gated on its dst-FIFO pop)
   // and is exercised on hardware where src/dst share VRAM.
   // ----------------------------------------------------------------------
   task test_dma_blend;

      localparam int          WPIX    = 20;                 // pixels/row (80B = 2 beats + 16B tail)
      localparam int          WBYTES  = WPIX * 4;            // 80
      localparam int          HROWS   = 2;
      localparam int          PITCH_B = 4 * 32;              // 128 (gap after each row)
      localparam logic [31:0] SRC_FB  = 32'h0000_0000;       // source over-layer  (u_src)
      localparam logic [31:0] DST_FB  = 32'h0000_8000;       // dest under-layer: read u_src, write u_dst
      localparam logic [31:0] SEQNO   = 32'hB1E0_0001;

      logic [AXI_DATA_WIDTH-1:0] sbeat, dbeat;
      logic [31:0] sp, dp, got, exp, rdata;
      logic [7:0]  sa;
      int          errors, r, p, nbeat, b, lane;

      begin
         nbeat = (WBYTES + 31) / 32;        // beats per row (ceil); last is partial

         // build the source over-layer and destination under-layer in u_src,
         // clear the u_dst rect (rows + gaps) so a missed/extra write shows up
         for (r = 0; r < HROWS; r++) begin
            for (b = 0; b < nbeat; b++) begin
               for (p = 0; p < 8; p++) begin
                  sa = 8'(p*37 + r*113 + b*5);                 // vary src alpha
                  sp = {sa, sa, sa >> 1, sa >> 2};             // premultiplied: rgb <= a
                  dp = {8'hFF, 8'(p*29 + r*61), 8'(p*7 + 13), 8'(p*53)}; // opaque under-layer
                  sbeat[p*32 +: 32] = sp;
                  dbeat[p*32 +: 32] = dp;
               end
               u_src.mem[(SRC_FB >> 5) + r*(PITCH_B >> 5) + b] = sbeat;
               u_src.mem[(DST_FB >> 5) + r*(PITCH_B >> 5) + b] = dbeat;
            end
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
         cmd_write(DMA_OP_OFF,       32'(DMA_OP_BLEND));

         $display("%t INFO: DMA BLEND doorbell (%0d px x %0d rows, %0dB tail)",
                  $time, WPIX, HROWS, WBYTES - (nbeat-1)*32);
         cmd_write(DMA_DOORBELL_OFF, SEQNO);

         wait (dma_irq);
         cmd_write(DMA_IRQ_OFF, 32'h1);
         cmd_read (DMA_FENCE_OFF, rdata);

         errors = 0;
         for (r = 0; r < HROWS; r++) begin
            // the WPIX composited pixels must equal the reference
            for (p = 0; p < WPIX; p++) begin
               b    = p / 8;
               lane = p % 8;
               sp  = u_src.mem[(SRC_FB >> 5) + r*(PITCH_B >> 5) + b][lane*32 +: 32];
               dp  = u_src.mem[(DST_FB >> 5) + r*(PITCH_B >> 5) + b][lane*32 +: 32];
               exp = ref_over(sp, dp);
               got = u_dst.mem[(DST_FB >> 5) + r*(PITCH_B >> 5) + b][lane*32 +: 32];
               if (got !== exp) begin
                  errors++;
                  if (errors <= 6)
                    $display("  row %0d px %0d MISMATCH got=%h exp=%h (s=%h d=%h)", r, p, got, exp, sp, dp);
               end
            end
            // pixels past WPIX in the masked tail beat must be untouched (0)
            for (p = WPIX; p < nbeat*8; p++) begin
               b    = p / 8;
               lane = p % 8;
               got  = u_dst.mem[(DST_FB >> 5) + r*(PITCH_B >> 5) + b][lane*32 +: 32];
               if (got !== '0) begin
                  errors++;
                  if (errors <= 6) $display("  row %0d px %0d tail OVERWRITTEN got=%h", r, p, got);
               end
            end
         end

         if (errors == 0 && rdata === SEQNO) begin
            $display("%t INFO: DMA BLEND verified (%0d px x %0d rows, premult SRC_OVER, fence=%h)",
                     $time, WPIX, HROWS, rdata);
            result = 1'b1;
         end
         else
           $display("%t ERROR: DMA BLEND failed (%0d mismatches, fence=%h)", $time, errors, rdata);
      end
   endtask // test_dma_blend

   // ----------------------------------------------------------------------
   // Unaligned BLEND: composite a source whose sub-beat byte offset differs
   // from the destination's. The source read stream is realigned
   // (vctrl_dma_realign) to the dst offset so source pixel k composites over
   // dst pixel k, and the write masks the partial first/last beats. One run:
   // verifies the @wpix composited pixels == the SW premult-over reference at
   // their byte-exact destination, and that dst bytes outside [dst,dst+width)
   // (head + tail, prefilled with a recognizable sentinel) are untouched.
   // Offsets are pixel-aligned (multiples of 4), as a cursor's always are.
   // ----------------------------------------------------------------------
   task automatic blend_unaligned_run(input int sbase, input int dbase,
                                      input int soff,  input int doff,
                                      input int wpix,  input int hrows,
                                      output bit ok);
      localparam int          PITCH = 512;              // 32-aligned; wide enough that
                                                        // adjacent rows never share a beat
      localparam logic [31:0] SENT  = 32'hDEAD_BEEF;    // dst head/tail prefill
      localparam logic [31:0] SEQNO = 32'hB1E0_0002;

      int          src_fb, dst_fb, wbytes, sbyte, dbyte, drow0;
      logic [7:0]  sa;
      logic [31:0] sp, dp, got, exp, rdata;
      int          errors, r, p, beat, lane;

      begin
         axi4k_violation = 1'b0;
         src_fb = sbase + soff;
         dst_fb = dbase + doff;
         wbytes = wpix * 4;
         errors = 0;

         for (r = 0; r < hrows; r++) begin
            // seed every dst beat the row touches with the sentinel (in u_dst);
            // init the same src/dst-underlayer beats in u_src to avoid X
            for (beat = (dst_fb + r*PITCH) >> 5;
                 beat <= (dst_fb + r*PITCH + wbytes - 1) >> 5; beat++)
              u_dst.mem[beat] = {8{SENT}};
            for (beat = (src_fb + r*PITCH) >> 5;
                 beat <= (src_fb + r*PITCH + wbytes - 1) >> 5; beat++)
              u_src.mem[beat] = '0;
            for (beat = (dst_fb + r*PITCH) >> 5;
                 beat <= (dst_fb + r*PITCH + wbytes - 1) >> 5; beat++)
              u_src.mem[beat] = '0;
            for (p = 0; p < wpix; p++) begin
               sa = 8'(p*37 + r*113);
               sp = {sa, sa, sa >> 1, sa >> 2};                       // premult src
               dp = {8'hFF, 8'(p*29 + r*61), 8'(p*7 + 13), 8'(p*53)}; // opaque dst
               sbyte = src_fb + r*PITCH + p*4;
               dbyte = dst_fb + r*PITCH + p*4;
               u_src.mem[sbyte >> 5][((sbyte >> 2) & 7)*32 +: 32] = sp;
               u_src.mem[dbyte >> 5][((dbyte >> 2) & 7)*32 +: 32] = dp;
            end
         end

         cmd_write(DMA_IRQEN_OFF,    32'h1);
         cmd_write(DMA_CTRL_OFF,     32'h1 << DMA_CTRL_ENABLE);
         cmd_write(DMA_SRC_OFF,      src_fb);
         cmd_write(DMA_DST_OFF,      dst_fb);
         cmd_write(DMA_SRCPITCH_OFF, PITCH);
         cmd_write(DMA_DSTPITCH_OFF, PITCH);
         cmd_write(DMA_WIDTH_OFF,    wbytes);
         cmd_write(DMA_HEIGHT_OFF,   hrows);
         cmd_write(DMA_OP_OFF,       32'(DMA_OP_BLEND));

         $display("%t INFO: BLEND unaligned src@%h dst@%h (soff=%0d doff=%0d %0d px x %0d rows)",
                  $time, src_fb, dst_fb, soff, doff, wpix, hrows);
         cmd_write(DMA_DOORBELL_OFF, SEQNO);

         wait (dma_irq);
         cmd_write(DMA_IRQ_OFF, 32'h1);
         cmd_read (DMA_FENCE_OFF, rdata);

         for (r = 0; r < hrows; r++) begin
            for (p = 0; p < wpix; p++) begin              // composited pixels
               sbyte = src_fb + r*PITCH + p*4;
               dbyte = dst_fb + r*PITCH + p*4;
               sp  = u_src.mem[sbyte >> 5][((sbyte >> 2) & 7)*32 +: 32];
               dp  = u_src.mem[dbyte >> 5][((dbyte >> 2) & 7)*32 +: 32];
               exp = ref_over(sp, dp);
               got = u_dst.mem[dbyte >> 5][((dbyte >> 2) & 7)*32 +: 32];
               if (got !== exp) begin
                  errors++;
                  if (errors <= 6)
                    $display("    row %0d px %0d MISMATCH got=%h exp=%h", r, p, got, exp);
               end
            end
            for (beat = (dst_fb + r*PITCH) >> 5;          // head/tail untouched
                 beat <= (dst_fb + r*PITCH + wbytes - 1) >> 5; beat++)
              for (lane = 0; lane < 8; lane++) begin
                 dbyte = beat*32 + lane*4;
                 if (dbyte < (dst_fb + r*PITCH) || dbyte >= (dst_fb + r*PITCH + wbytes)) begin
                    got = u_dst.mem[beat][lane*32 +: 32];
                    if (got !== SENT) begin
                       errors++;
                       if (errors <= 6)
                         $display("    row %0d beat %0d lane %0d OVERWRITTEN got=%h", r, beat, lane, got);
                    end
                 end
              end
         end

         ok = (errors == 0) && (rdata === SEQNO) && !axi4k_violation;
         $display("%t %s: BLEND unaligned soff=%0d doff=%0d (%0d errors, fence=%h, viol=%b)",
                  $time, ok ? "INFO" : "ERROR", soff, doff, errors, rdata, axi4k_violation);
      end
   endtask // blend_unaligned_run

   task automatic test_dma_blend_unaligned;
      bit ok, all_ok;
      begin
         all_ok = 1'b1;
         blend_unaligned_run('h0000, 'h8000,  4, 20, 20, 2, ok); all_ok &= ok; // shift=16, +head beat
         blend_unaligned_run('h0000, 'h8000,  0, 16, 17, 2, ok); all_ok &= ok; // src aligned, dst offset
         blend_unaligned_run('h0000, 'h8000, 28,  4, 10, 2, ok); all_ok &= ok; // soff>doff (look-ahead)
         blend_unaligned_run('h0000, 'h8000,  0, 28,  5, 1, ok); all_ok &= ok; // tiny head, single short row
         blend_unaligned_run('h0000, 'h8000,  8,  8, 20, 2, ok); all_ok &= ok; // shift=0 (regression)
         blend_unaligned_run('h0F40, 'h8F40,  0, 16, 64, 2, ok); all_ok &= ok; // source crosses 4 KiB
         result = all_ok;
         $display("%t %s: DMA BLEND unaligned suite", $time, all_ok ? "INFO" : "ERROR");
      end
   endtask // test_dma_blend_unaligned
