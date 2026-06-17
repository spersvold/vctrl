// -*- mode: verilog; mode: font-lock; indent-tabs-mode: nil -*-
// vi: set et ts=3 sw=3 sts=3:
//
// Copyright 2026 Steffen Persvold
// SPDX-License-Identifier: Apache-2.0
//
// ========================================================================
// File        : tb.sv
// Author      : Steffen Persvold
// Created     : May 16, 2026
// ========================================================================
// Description : Top testbench
// ========================================================================
//

`ifndef UNIQUE_TAG
 `define UNIQUE_TAG "NDf0EjIxjsZbmheYe4u"
`endif

`ifndef TESTNAME
 `define TESTNAME "vctrl_core"
`endif

module tb
  ();

   timeunit 1ns;
   timeprecision 1ps;

   event done;
   bit   result;

   initial begin
      // this will cause %t to show simulation times using ns
      $timeformat(-9,3," ns",13);
      if ($test$plusargs("DUMP")) begin
         $display("%t INFO: module=%m, starting dumpfile", $time);
`ifdef HAVE_VCDPLUSON
         $vcdpluson;
`else
         $dumpfile("tb.fst");
         $dumpvars(0, tb);
`endif
      end
   end

   import vctrl_pkg::*;

   //

   logic                           clk_sys = 1'b0;
   logic                           rst_sys = 1'b1;

   parameter real CLK_SYS_FREQ = 100.0; // MHz
   localparam CLK_SYS_PERIOD = 1000.0 / CLK_SYS_FREQ; // in ns

   always #(CLK_SYS_PERIOD/2) clk_sys = ~clk_sys;

   //

   logic                           clk_pix = 1'b0;
   logic                           rst_pix = 1'b1;

   localparam real CLK_PIX_FREQ = 25.175; // MHz
   localparam CLK_PIX_PERIOD = 1000.0 / CLK_PIX_FREQ; // in ns

   always #(CLK_PIX_PERIOD/2) clk_pix = ~clk_pix;

   //

   parameter integer VR_SIZE  = 1920 * 1080 * 4; // Max size of Frame Buffer memory (bytes)
   parameter integer VR_DATAW = 32;              // Frame Buffer Data Width (bits)
   parameter integer LB_DEPTH = 2048;            // Line Buffer depth (in pixels), i.e this is effectively
                                                 // your max horizontal resolution

   localparam integer VR_ADDRW = $clog2(VR_SIZE);

   logic                           cfg_req;      // Config request
   logic [11:2]                    cfg_adr;      // Config request address
   logic                           cfg_we;       // Config request write enable
   logic [ 3:0]                    cfg_be;       // Config request byte enable
   logic [31:0]                    cfg_d;        // Config request write data
   logic [31:0]                    cfg_q;        // Config request read data
   logic                           cfg_ack;      // Config request acknowledge

   logic                           irq;          // Interrupt
   logic                           frame_sys;    // Start of new frame (in system clock domain)
   logic [31:2]                    vbar;         // Video base address (scanout base)
   logic [31:2]                    vsiz;         // Scanout buffer size (bytes); bounds the fetch
   logic                           ven;          // Scanout enable (CTRL.VEN); gates the fetch
   logic                           fetch_idle;   // Scanout fetch idle (no reads in flight)

   logic                           fb_rdreq;     // Memory read request
   logic [VR_ADDRW -1:0]           fb_raddr;     // Memory read address
   logic                           fb_rdack;     // Memory read ackowledge
   logic [VR_DATAW -1:0]           fb_rdata;     // Memory read data
   logic                           fb_rvalid;    // Memory read data valid

   // PLL reconfiguration interface (to/from board-level hdmi_pll_recfg)
   plldivcnt_t                     pll_divcnt;   // PLLDIVCNT (logical M/N/C)
   logic                           pll_apply;    // reconfig trigger pulse (clk_sys)
   logic                           pll_done;     // reconfig done pulse (clk_sys)
   logic                           pll_locked;   // synchronized PLL locked
   logic                           pll_error;    // synchronized recal error

   logic [ 7:0]                    vga_r;
   logic [ 7:0]                    vga_g;
   logic [ 7:0]                    vga_b;
   logic                           vga_bl;
   logic                           vga_hs;
   logic                           vga_vs;

   //

   // Instantiate DUT
   vctrl_core #
     (.VR_SIZE  (VR_SIZE),
      .VR_DATAW (VR_DATAW),
      .LB_DEPTH (LB_DEPTH))
   dut
     (.*);


    // ----------------------------------------------------------------------
    // AXI4 read master (write channel tied idle)
    // ----------------------------------------------------------------------
   localparam AXI_ID_WIDTH   = 1;
   localparam AXI_ADDR_WIDTH = 32;
   localparam AXI_DATA_WIDTH = 256;
   localparam AXI_STRB_WIDTH = AXI_DATA_WIDTH/8;

   logic [AXI_ID_WIDTH  -1:0]      m_axi_arid;
   logic [AXI_ADDR_WIDTH-1:0]      m_axi_araddr;
   logic [7:0]                     m_axi_arlen;
   logic [2:0]                     m_axi_arsize;
   logic [1:0]                     m_axi_arburst;
   logic                           m_axi_arlock;
   logic [3:0]                     m_axi_arcache;
   logic [2:0]                     m_axi_arprot;
   logic                           m_axi_arvalid;
   logic                           m_axi_arready;
   logic [AXI_ID_WIDTH-1:0]        m_axi_rid;
   logic [AXI_DATA_WIDTH-1:0]      m_axi_rdata;
   logic [1:0]                     m_axi_rresp;
   logic                           m_axi_rlast;
   logic                           m_axi_rvalid;
   logic                           m_axi_rready;
   //
   logic [AXI_ID_WIDTH-1:0]        m_axi_awid;
   logic [AXI_ADDR_WIDTH-1:0]      m_axi_awaddr;
   logic [7:0]                     m_axi_awlen;
   logic [2:0]                     m_axi_awsize;
   logic [1:0]                     m_axi_awburst;
   logic                           m_axi_awlock;
   logic [3:0]                     m_axi_awcache;
   logic [2:0]                     m_axi_awprot;
   logic                           m_axi_awvalid;
   logic                           m_axi_awready;
   logic [AXI_DATA_WIDTH-1:0]      m_axi_wdata;
   logic [AXI_STRB_WIDTH-1:0]      m_axi_wstrb;
   logic                           m_axi_wlast;
   logic                           m_axi_wvalid;
   logic                           m_axi_wready;
   logic [AXI_ID_WIDTH-1:0]        m_axi_bid;
   logic [1:0]                     m_axi_bresp;
   logic                           m_axi_bvalid;
   logic                           m_axi_bready;

   vctrl_axim #
     (.ADDR_WIDTH     (AXI_ADDR_WIDTH),
      .AXI_DATA_WIDTH (AXI_DATA_WIDTH),
      .FB_DATA_WIDTH  (VR_DATAW),
      .FB_ADDR_WIDTH  (VR_ADDRW),
      .ID_WIDTH       (AXI_ID_WIDTH),
      .BURST_LEN      (16),
      .FIFO_LGDEPTH   (9))
   u_axim
     (.clk         (clk_sys),
      .rst         (rst_sys),
      .fb_base     ({vbar, 2'b00}),
      .fb_size     ({vsiz, 2'b00}),
      // others
      .*);

   axi_ram #
     (.DATA_WIDTH (AXI_DATA_WIDTH),
      .ADDR_WIDTH (AXI_ADDR_WIDTH),
      .STRB_WIDTH (AXI_STRB_WIDTH),
      .ID_WIDTH   (AXI_ID_WIDTH),
      .PIPELINE_OUTPUT (1))
   u_ram
     (.clk           (clk_sys),
      .rst           (rst_sys),
      .s_axi_awid    (m_axi_awid),
      .s_axi_awaddr  (m_axi_awaddr),
      .s_axi_awlen   (m_axi_awlen),
      .s_axi_awsize  (m_axi_awsize),
      .s_axi_awburst (m_axi_awburst),
      .s_axi_awlock  (m_axi_awlock),
      .s_axi_awcache (m_axi_awcache),
      .s_axi_awprot  (m_axi_awprot),
      .s_axi_awvalid (m_axi_awvalid),
      .s_axi_awready (m_axi_awready),
      .s_axi_wdata   (m_axi_wdata),
      .s_axi_wstrb   (m_axi_wstrb),
      .s_axi_wlast   (m_axi_wlast),
      .s_axi_wvalid  (m_axi_wvalid),
      .s_axi_wready  (m_axi_wready),
      .s_axi_bid     (m_axi_bid),
      .s_axi_bresp   (m_axi_bresp),
      .s_axi_bvalid  (m_axi_bvalid),
      .s_axi_bready  (m_axi_bready),
      .s_axi_arid    (m_axi_arid),
      .s_axi_araddr  (m_axi_araddr),
      .s_axi_arlen   (m_axi_arlen),
      .s_axi_arsize  (m_axi_arsize),
      .s_axi_arburst (m_axi_arburst),
      .s_axi_arlock  (m_axi_arlock),
      .s_axi_arcache (m_axi_arcache),
      .s_axi_arprot  (m_axi_arprot),
      .s_axi_arvalid (m_axi_arvalid),
      .s_axi_arready (m_axi_arready),
      .s_axi_rid     (m_axi_rid),
      .s_axi_rdata   (m_axi_rdata),
      .s_axi_rresp   (m_axi_rresp),
      .s_axi_rlast   (m_axi_rlast),
      .s_axi_rvalid  (m_axi_rvalid),
      .s_axi_rready  (m_axi_rready)
      );

   // ----------------------------------------------------------------------
   // DMA engine DUT (+TESTCASE=dma) : source and destination AXI memories.
   // A small 24-bit address space keeps each axi_ram model at 16 MiB.
   // ----------------------------------------------------------------------
   localparam DMA_ADDR_WIDTH = 24;
   localparam DMA_RESP_LAT   = 12;   // fixed source/dest memory response latency (cycles)

   logic                       cmd_req;
   logic [11: 2]               cmd_adr;
   logic                       cmd_we;
   logic [ 3: 0]               cmd_be;
   logic [31: 0]               cmd_d;
   logic [31: 0]               cmd_q;
   logic                       cmd_ack;
   logic                       dma_irq;

   initial begin
      cmd_req = 1'b0; cmd_we = 1'b0; cmd_be = '0; cmd_adr = '0; cmd_d = '0;
   end

   // source read master <-> u_src
   logic [AXI_ID_WIDTH  -1:0]  rd_arid;
   logic [DMA_ADDR_WIDTH-1:0]  rd_araddr;
   logic [7:0]                 rd_arlen;
   logic [2:0]                 rd_arsize;
   logic [1:0]                 rd_arburst;
   logic                       rd_arlock;
   logic [3:0]                 rd_arcache;
   logic [2:0]                 rd_arprot;
   logic                       rd_arvalid, rd_arready;
   logic [AXI_ID_WIDTH  -1:0]  rd_rid;
   logic [AXI_DATA_WIDTH-1:0]  rd_rdata;
   logic [1:0]                 rd_rresp;
   logic                       rd_rlast, rd_rvalid, rd_rready;

   // destination write master <-> u_dst
   logic [AXI_ID_WIDTH  -1:0]  wr_awid;
   logic [DMA_ADDR_WIDTH-1:0]  wr_awaddr;
   logic [7:0]                 wr_awlen;
   logic [2:0]                 wr_awsize;
   logic [1:0]                 wr_awburst;
   logic                       wr_awlock;
   logic [3:0]                 wr_awcache;
   logic [2:0]                 wr_awprot;
   logic                       wr_awvalid, wr_awready;
   logic [AXI_DATA_WIDTH-1:0]  wr_wdata;
   logic [AXI_STRB_WIDTH-1:0]  wr_wstrb;
   logic                       wr_wlast, wr_wvalid, wr_wready;
   logic [AXI_ID_WIDTH  -1:0]  wr_bid;
   logic [1:0]                 wr_bresp;
   logic                       wr_bvalid, wr_bready;

   vctrl_dma #
     (.ADDR_WIDTH     (DMA_ADDR_WIDTH),
      .AXI_DATA_WIDTH (AXI_DATA_WIDTH),
      .ID_WIDTH       (AXI_ID_WIDTH),
      .BURST_LEN      (16),
      .FIFO_LGDEPTH   (9))
   u_dma
     (.clk_sys          (clk_sys),
      .rst_sys          (rst_sys),
      .cmd_req          (cmd_req),
      .cmd_adr          (cmd_adr),
      .cmd_we           (cmd_we),
      .cmd_be           (cmd_be),
      .cmd_d            (cmd_d),
      .cmd_q            (cmd_q),
      .cmd_ack          (cmd_ack),
      .irq              (dma_irq),
      .m_axi_rd_arid    (rd_arid),
      .m_axi_rd_araddr  (rd_araddr),
      .m_axi_rd_arlen   (rd_arlen),
      .m_axi_rd_arsize  (rd_arsize),
      .m_axi_rd_arburst (rd_arburst),
      .m_axi_rd_arlock  (rd_arlock),
      .m_axi_rd_arcache (rd_arcache),
      .m_axi_rd_arprot  (rd_arprot),
      .m_axi_rd_arvalid (rd_arvalid),
      .m_axi_rd_arready (rd_arready),
      .m_axi_rd_rid     (rd_rid),
      .m_axi_rd_rdata   (rd_rdata),
      .m_axi_rd_rresp   (rd_rresp),
      .m_axi_rd_rlast   (rd_rlast),
      .m_axi_rd_rvalid  (rd_rvalid),
      .m_axi_rd_rready  (rd_rready),
      .m_axi_wr_awid    (wr_awid),
      .m_axi_wr_awaddr  (wr_awaddr),
      .m_axi_wr_awlen   (wr_awlen),
      .m_axi_wr_awsize  (wr_awsize),
      .m_axi_wr_awburst (wr_awburst),
      .m_axi_wr_awlock  (wr_awlock),
      .m_axi_wr_awcache (wr_awcache),
      .m_axi_wr_awprot  (wr_awprot),
      .m_axi_wr_awvalid (wr_awvalid),
      .m_axi_wr_awready (wr_awready),
      .m_axi_wr_wdata   (wr_wdata),
      .m_axi_wr_wstrb   (wr_wstrb),
      .m_axi_wr_wlast   (wr_wlast),
      .m_axi_wr_wvalid  (wr_wvalid),
      .m_axi_wr_wready  (wr_wready),
      .m_axi_wr_bid     (wr_bid),
      .m_axi_wr_bresp   (wr_bresp),
      .m_axi_wr_bvalid  (wr_bvalid),
      .m_axi_wr_bready  (wr_bready));

   axi_ram #
     (.DATA_WIDTH      (AXI_DATA_WIDTH),
      .ADDR_WIDTH      (DMA_ADDR_WIDTH),
      .STRB_WIDTH      (AXI_STRB_WIDTH),
      .ID_WIDTH        (AXI_ID_WIDTH),
      .PIPELINE_OUTPUT (1),
      .RESP_LATENCY    (DMA_RESP_LAT))
   u_src   // answers the read master; write side idle
     (.clk           (clk_sys),
      .rst           (rst_sys),
      .s_axi_awid    ('0),
      .s_axi_awaddr  ('0),
      .s_axi_awlen   ('0),
      .s_axi_awsize  ('0),
      .s_axi_awburst ('0),
      .s_axi_awlock  ('0),
      .s_axi_awcache ('0),
      .s_axi_awprot  ('0),
      .s_axi_awvalid (1'b0),
      .s_axi_awready (),
      .s_axi_wdata   ('0),
      .s_axi_wstrb   ('0),
      .s_axi_wlast   (1'b0),
      .s_axi_wvalid  (1'b0),
      .s_axi_wready  (),
      .s_axi_bid     (),
      .s_axi_bresp   (),
      .s_axi_bvalid  (),
      .s_axi_bready  (1'b1),
      .s_axi_arid    (rd_arid),
      .s_axi_araddr  (rd_araddr),
      .s_axi_arlen   (rd_arlen),
      .s_axi_arsize  (rd_arsize),
      .s_axi_arburst (rd_arburst),
      .s_axi_arlock  (rd_arlock),
      .s_axi_arcache (rd_arcache),
      .s_axi_arprot  (rd_arprot),
      .s_axi_arvalid (rd_arvalid),
      .s_axi_arready (rd_arready),
      .s_axi_rid     (rd_rid),
      .s_axi_rdata   (rd_rdata),
      .s_axi_rresp   (rd_rresp),
      .s_axi_rlast   (rd_rlast),
      .s_axi_rvalid  (rd_rvalid),
      .s_axi_rready  (rd_rready));

   axi_ram #
     (.DATA_WIDTH      (AXI_DATA_WIDTH),
      .ADDR_WIDTH      (DMA_ADDR_WIDTH),
      .STRB_WIDTH      (AXI_STRB_WIDTH),
      .ID_WIDTH        (AXI_ID_WIDTH),
      .PIPELINE_OUTPUT (1),
      .RESP_LATENCY    (DMA_RESP_LAT))
   u_dst   // answers the write master; read side idle
     (.clk           (clk_sys),
      .rst           (rst_sys),
      .s_axi_awid    (wr_awid),
      .s_axi_awaddr  (wr_awaddr),
      .s_axi_awlen   (wr_awlen),
      .s_axi_awsize  (wr_awsize),
      .s_axi_awburst (wr_awburst),
      .s_axi_awlock  (wr_awlock),
      .s_axi_awcache (wr_awcache),
      .s_axi_awprot  (wr_awprot),
      .s_axi_awvalid (wr_awvalid),
      .s_axi_awready (wr_awready),
      .s_axi_wdata   (wr_wdata),
      .s_axi_wstrb   (wr_wstrb),
      .s_axi_wlast   (wr_wlast),
      .s_axi_wvalid  (wr_wvalid),
      .s_axi_wready  (wr_wready),
      .s_axi_bid     (wr_bid),
      .s_axi_bresp   (wr_bresp),
      .s_axi_bvalid  (wr_bvalid),
      .s_axi_bready  (wr_bready),
      .s_axi_arid    ('0),
      .s_axi_araddr  ('0),
      .s_axi_arlen   ('0),
      .s_axi_arsize  ('0),
      .s_axi_arburst ('0),
      .s_axi_arlock  ('0),
      .s_axi_arcache ('0),
      .s_axi_arprot  ('0),
      .s_axi_arvalid (1'b0),
      .s_axi_arready (),
      .s_axi_rid     (),
      .s_axi_rdata   (),
      .s_axi_rresp   (),
      .s_axi_rlast   (),
      .s_axi_rvalid  (),
      .s_axi_rready  (1'b0));

   //

   longint unsigned expectedRunTime;
   initial expectedRunTime = 1_000;

   initial begin
      fork
         begin: testBlock
            runtest;
         end
         begin: timeoutBlock
            while ($realtime < expectedRunTime) #1000;
            $display("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
            $display("+ ERROR: Test has exceeded timeout value expectedRunTime");
            $display("+ Test finished at time %t", $realtime);
            $display("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
            $display("");
         end
      join_any
      #200;
      #200;
      $finish;
   end

   // Per-testcase stimulus lives in its own include; runtest below brings
   // the DUT out of reset and dispatches one via +TESTCASE=<name>.
   `include "tb_test_scanout.svh"
   `include "tb_test_dma.svh"

   task runtest;

      string testcase;

      begin
         result = 1'b0;
         expectedRunTime = 64'd1_000_000_000;

         #100;

         @(posedge clk_sys) #0.1ns;
         rst_sys = 1'b0;

         #100;

         @(posedge clk_pix) #0.1ns;
         rst_pix = 1'b0;

         #100;

         if (!$value$plusargs("TESTCASE=%s", testcase))
           testcase = "scanout";
         $display("%t INFO: Running testcase '%s'", $time, testcase);

         case (testcase)
           "scanout": test_scanout;
           "dma"    : test_dma;
           default  : $error("unknown TESTCASE '%s'", testcase);
         endcase

         #100ns;
         ->done;
      end
   endtask

   always @done begin
      #1;
      $display(" ");
      $display("=================================================================================");
      $display("= %s = Final Test Result: %s = %s", `TESTNAME,
               ((result !== 1'b1) ? "failed" : "passed"), `UNIQUE_TAG);
      $display("=================================================================================");
      $finish;
   end

endmodule // tb
