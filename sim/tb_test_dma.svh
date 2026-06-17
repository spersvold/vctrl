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
