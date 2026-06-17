// -*- mode: verilog; mode: font-lock; indent-tabs-mode: nil -*-
// vi: set et ts=3 sw=3 sts=3:
//
// Copyright 2026 Steffen Persvold
// SPDX-License-Identifier: Apache-2.0
//
// ========================================================================
// File        : axi_ram.sv
// Author      : Steffen Persvold
// Created     : May 16, 2026
// ========================================================================
// Description : AXI4 RAM model
// ========================================================================
//

module axi_ram
 #(
   // Width of data bus in bits
   parameter DATA_WIDTH = 32,
   // Width of address bus in bits
   parameter ADDR_WIDTH = 16,
   // Width of wstrb (width of data bus in words)
   parameter STRB_WIDTH = (DATA_WIDTH/8),
   // Width of ID signal
   parameter ID_WIDTH = 8,
   // Extra pipeline register on output
   parameter PIPELINE_OUTPUT = 0,
   // Fixed response latency (cycles) added to the R and B channels; 0 = off.
   // Assumes the master holds R/B ready high (vctrl_axim and vctrl_dma_rd do):
   // it delays the response stream, which also makes the model accept multiple
   // outstanding bursts -- so it exercises latency hiding, not just slowdown.
   parameter RESP_LATENCY = 0
   )
  (
   input  logic                   clk,
   input  logic                   rst,

   input  logic [ID_WIDTH-1:0]    s_axi_awid,
   input  logic [ADDR_WIDTH-1:0]  s_axi_awaddr,
   input  logic [7:0]             s_axi_awlen,
   input  logic [2:0]             s_axi_awsize,
   input  logic [1:0]             s_axi_awburst,
   input  logic                   s_axi_awlock,
   input  logic [3:0]             s_axi_awcache,
   input  logic [2:0]             s_axi_awprot,
   input  logic                   s_axi_awvalid,
   output logic                   s_axi_awready,
   input  logic [DATA_WIDTH-1:0]  s_axi_wdata,
   input  logic [STRB_WIDTH-1:0]  s_axi_wstrb,
   input  logic                   s_axi_wlast,
   input  logic                   s_axi_wvalid,
   output logic                   s_axi_wready,
   output logic [ID_WIDTH-1:0]    s_axi_bid,
   output logic [1:0]             s_axi_bresp,
   output logic                   s_axi_bvalid,
   input  logic                   s_axi_bready,
   input  logic [ID_WIDTH-1:0]    s_axi_arid,
   input  logic [ADDR_WIDTH-1:0]  s_axi_araddr,
   input  logic [7:0]             s_axi_arlen,
   input  logic [2:0]             s_axi_arsize,
   input  logic [1:0]             s_axi_arburst,
   input  logic                   s_axi_arlock,
   input  logic [3:0]             s_axi_arcache,
   input  logic [2:0]             s_axi_arprot,
   input  logic                   s_axi_arvalid,
   output logic                   s_axi_arready,
   output logic [ID_WIDTH-1:0]    s_axi_rid,
   output logic [DATA_WIDTH-1:0]  s_axi_rdata,
   output logic [1:0]             s_axi_rresp,
   output logic                   s_axi_rlast,
   output logic                   s_axi_rvalid,
   input  logic                   s_axi_rready
   );

   parameter VALID_ADDR_WIDTH = ADDR_WIDTH - $clog2(STRB_WIDTH);
   parameter WORD_WIDTH = STRB_WIDTH;
   parameter WORD_SIZE = DATA_WIDTH/WORD_WIDTH;

   // bus width assertions
   initial begin
      if (WORD_SIZE * STRB_WIDTH != DATA_WIDTH) begin
         $error("Error: AXI data width not evenly divisble (instance %m)");
         $finish;
      end

      if (2**$clog2(WORD_WIDTH) != WORD_WIDTH) begin
         $error("Error: AXI word width must be even power of two (instance %m)");
         $finish;
      end
   end

   localparam [0:0]
     READ_STATE_IDLE = 1'd0,
     READ_STATE_BURST = 1'd1;

   logic [0:0] read_state_reg = READ_STATE_IDLE, read_state_next;

   localparam [1:0]
     WRITE_STATE_IDLE = 2'd0,
     WRITE_STATE_BURST = 2'd1,
     WRITE_STATE_RESP = 2'd2;

   logic [1:0] write_state_reg = WRITE_STATE_IDLE, write_state_next;

   logic                  mem_wr_en;
   logic                  mem_rd_en;

   typedef logic [2:0] axsize_t;

   logic [ID_WIDTH  -1:0] read_id_reg = {ID_WIDTH{1'b0}}, read_id_next;
   logic [ADDR_WIDTH-1:0] read_addr_reg = {ADDR_WIDTH{1'b0}}, read_addr_next;
   logic [ADDR_WIDTH-1:0] read_first_addr_reg = {ADDR_WIDTH{1'b0}}, read_first_addr_next;
   logic [ADDR_WIDTH-1:0] read_last_addr_reg = {ADDR_WIDTH{1'b0}}, read_last_addr_next;
   logic [ADDR_WIDTH-1:0] read_wrap_addr_reg = {ADDR_WIDTH{1'b0}}, read_wrap_addr_next;
   logic [7:0]            read_count_reg = 8'd0, read_count_next;
   logic [2:0]            read_size_reg = 3'd0, read_size_next;
   logic [1:0]            read_burst_reg = 2'd0, read_burst_next;
   logic [ID_WIDTH  -1:0] write_id_reg = {ID_WIDTH{1'b0}}, write_id_next;
   logic [ADDR_WIDTH-1:0] write_addr_reg = {ADDR_WIDTH{1'b0}}, write_addr_next;
   logic [ADDR_WIDTH-1:0] write_first_addr_reg = {ADDR_WIDTH{1'b0}}, write_first_addr_next;
   logic [ADDR_WIDTH-1:0] write_last_addr_reg = {ADDR_WIDTH{1'b0}}, write_last_addr_next;
   logic [ADDR_WIDTH-1:0] write_wrap_addr_reg = {ADDR_WIDTH{1'b0}}, write_wrap_addr_next;
   logic [7:0]            write_count_reg = 8'd0, write_count_next;
   logic [2:0]            write_size_reg = 3'd0, write_size_next;
   logic [1:0]            write_burst_reg = 2'd0, write_burst_next;

   logic                  s_axi_awready_reg = 1'b0, s_axi_awready_next;
   logic                  s_axi_wready_reg = 1'b0, s_axi_wready_next;
   logic [ID_WIDTH  -1:0] s_axi_bid_reg = {ID_WIDTH{1'b0}}, s_axi_bid_next;
   logic                  s_axi_bvalid_reg = 1'b0, s_axi_bvalid_next;
   logic                  s_axi_arready_reg = 1'b0, s_axi_arready_next;
   logic [ID_WIDTH  -1:0] s_axi_rid_reg = {ID_WIDTH{1'b0}}, s_axi_rid_next;
   logic [DATA_WIDTH-1:0] s_axi_rdata_reg = {DATA_WIDTH{1'b0}}, s_axi_rdata_next;
   logic                  s_axi_rlast_reg = 1'b0, s_axi_rlast_next;
   logic                  s_axi_rvalid_reg = 1'b0, s_axi_rvalid_next;
   logic [ID_WIDTH  -1:0] s_axi_rid_pipe_reg = {ID_WIDTH{1'b0}};
   logic [DATA_WIDTH-1:0] s_axi_rdata_pipe_reg = {DATA_WIDTH{1'b0}};
   logic                  s_axi_rlast_pipe_reg = 1'b0;
   logic                  s_axi_rvalid_pipe_reg = 1'b0;

   logic [DATA_WIDTH-1:0] mem[(2**VALID_ADDR_WIDTH)-1:0];

   wire [VALID_ADDR_WIDTH-1:0] s_axi_awaddr_valid = VALID_ADDR_WIDTH'(s_axi_awaddr >> (ADDR_WIDTH - VALID_ADDR_WIDTH));
   wire [VALID_ADDR_WIDTH-1:0] s_axi_araddr_valid = VALID_ADDR_WIDTH'(s_axi_araddr >> (ADDR_WIDTH - VALID_ADDR_WIDTH));
   wire [VALID_ADDR_WIDTH-1:0] read_addr_valid = VALID_ADDR_WIDTH'(read_addr_reg >> (ADDR_WIDTH - VALID_ADDR_WIDTH));
   wire [VALID_ADDR_WIDTH-1:0] write_addr_valid = VALID_ADDR_WIDTH'(write_addr_reg >> (ADDR_WIDTH - VALID_ADDR_WIDTH));

   assign s_axi_awready = s_axi_awready_reg;
   assign s_axi_wready  = s_axi_wready_reg;
   assign s_axi_arready = s_axi_arready_reg;
   assign s_axi_bresp   = 2'b00; //AXRESP_OKAY;
   assign s_axi_rresp   = 2'b00; //AXRESP_OKAY;

   // FSM response outputs (pre-latency)
   wire                  rvalid_pre = PIPELINE_OUTPUT ? s_axi_rvalid_pipe_reg : s_axi_rvalid_reg;
   wire [ID_WIDTH-1:0]   rid_pre    = PIPELINE_OUTPUT ? s_axi_rid_pipe_reg    : s_axi_rid_reg;
   wire [DATA_WIDTH-1:0] rdata_pre  = PIPELINE_OUTPUT ? s_axi_rdata_pipe_reg  : s_axi_rdata_reg;
   wire                  rlast_pre  = PIPELINE_OUTPUT ? s_axi_rlast_pipe_reg  : s_axi_rlast_reg;

   generate
     if (RESP_LATENCY == 0) begin : g_nolat
        assign s_axi_rvalid = rvalid_pre;
        assign s_axi_rid    = rid_pre;
        assign s_axi_rdata  = rdata_pre;
        assign s_axi_rlast  = rlast_pre;
        assign s_axi_bvalid = s_axi_bvalid_reg;
        assign s_axi_bid    = s_axi_bid_reg;
     end
     else begin : g_lat
        // Delay the response payload (valid + fields) RESP_LATENCY cycles via a
        // chain of register stages (stage[0] = live FSM output). The master is
        // assumed to hold ready high, so no backpressure handling is needed.
        localparam int RW = 1 + ID_WIDTH + DATA_WIDTH + 1;  // {valid,id,data,last}
        localparam int BW = 1 + ID_WIDTH;                   // {valid,id}
        logic [RW-1:0] r_stg [0:RESP_LATENCY];
        logic [BW-1:0] b_stg [0:RESP_LATENCY];

        assign r_stg[0] = {rvalid_pre, rid_pre, rdata_pre, rlast_pre};
        assign b_stg[0] = {s_axi_bvalid_reg, s_axi_bid_reg};

        for (genvar gi = 0; gi < RESP_LATENCY; gi++) begin : g_stage
           always_ff @(posedge clk)
             if (rst) begin
                r_stg[gi+1] <= '0;
                b_stg[gi+1] <= '0;
             end
             else begin
                r_stg[gi+1] <= r_stg[gi];
                b_stg[gi+1] <= b_stg[gi];
             end
        end

        assign {s_axi_rvalid, s_axi_rid, s_axi_rdata, s_axi_rlast} = r_stg[RESP_LATENCY];
        assign {s_axi_bvalid, s_axi_bid}                          = b_stg[RESP_LATENCY];
     end
   endgenerate

   always_comb begin
      write_state_next = WRITE_STATE_IDLE;

      mem_wr_en = 1'b0;

      write_id_next = write_id_reg;
      write_addr_next = write_addr_reg;
      write_first_addr_next = write_first_addr_reg;
      write_last_addr_next = write_last_addr_reg;
      write_wrap_addr_next = write_wrap_addr_reg;
      write_count_next = write_count_reg;
      write_size_next = write_size_reg;
      write_burst_next = write_burst_reg;

      s_axi_awready_next = 1'b0;
      s_axi_wready_next = 1'b0;
      s_axi_bid_next = s_axi_bid_reg;
      s_axi_bvalid_next = s_axi_bvalid_reg && !s_axi_bready;

      unique case (write_state_reg)
        WRITE_STATE_IDLE: begin
           s_axi_awready_next = 1'b1;

           if (s_axi_awready && s_axi_awvalid) begin
              write_id_next = s_axi_awid;
              write_addr_next = s_axi_awaddr;
              write_first_addr_next = s_axi_awaddr;
              write_count_next = s_axi_awlen;
              write_size_next = s_axi_awsize < axsize_t'($clog2(STRB_WIDTH)) ? s_axi_awsize : axsize_t'($clog2(STRB_WIDTH));
              write_burst_next = s_axi_awburst;
              write_wrap_addr_next = s_axi_awaddr & ~(((ADDR_WIDTH'(1) << write_size_next) * (ADDR_WIDTH'(s_axi_awlen) + ADDR_WIDTH'(1))) - ADDR_WIDTH'(1));
              write_last_addr_next = write_wrap_addr_next + ((ADDR_WIDTH'(1) << write_size_next) * (ADDR_WIDTH'(s_axi_awlen) + ADDR_WIDTH'(1))) - ADDR_WIDTH'(1);

              s_axi_awready_next = 1'b0;
              s_axi_wready_next = 1'b1;
              write_state_next = WRITE_STATE_BURST;
           end else begin
              write_state_next = WRITE_STATE_IDLE;
           end
        end
        WRITE_STATE_BURST: begin
           s_axi_wready_next = 1'b1;

           if (s_axi_wready && s_axi_wvalid) begin
              mem_wr_en = 1'b1;
              if (write_burst_reg != 2'b00) begin
                 write_addr_next = write_addr_reg + (1 << write_size_reg);
              end
              if (write_burst_reg == 2'b10) begin
                 if (write_addr_next > write_last_addr_reg) begin
                    write_addr_next = write_wrap_addr_reg;
                 end
              end
              write_count_next = write_count_reg - 1;
              if (write_count_reg > 0) begin
                 write_state_next = WRITE_STATE_BURST;
              end else begin
                 s_axi_wready_next = 1'b0;
                 if (s_axi_bready || !s_axi_bvalid) begin
                    s_axi_bid_next = write_id_reg;
                    s_axi_bvalid_next = 1'b1;
                    s_axi_awready_next = 1'b1;
                    write_state_next = WRITE_STATE_IDLE;
                 end else begin
                    write_state_next = WRITE_STATE_RESP;
                 end
              end
           end else begin
              write_state_next = WRITE_STATE_BURST;
           end
        end
        WRITE_STATE_RESP: begin
           if (s_axi_bready || !s_axi_bvalid) begin
              s_axi_bid_next = write_id_reg;
              s_axi_bvalid_next = 1'b1;
              s_axi_awready_next = 1'b1;
              write_state_next = WRITE_STATE_IDLE;
           end else begin
              write_state_next = WRITE_STATE_RESP;
           end
        end
        default: ;
      endcase
   end

   always @(posedge clk) begin
      write_state_reg <= write_state_next;

      write_id_reg <= write_id_next;
      write_addr_reg <= write_addr_next;
      write_first_addr_reg <= write_first_addr_next;
      write_last_addr_reg <= write_last_addr_next;
      write_wrap_addr_reg <= write_wrap_addr_next;
      write_count_reg <= write_count_next;
      write_size_reg <= write_size_next;
      write_burst_reg <= write_burst_next;

      s_axi_awready_reg <= s_axi_awready_next;
      s_axi_wready_reg <= s_axi_wready_next;
      s_axi_bid_reg <= s_axi_bid_next;
      s_axi_bvalid_reg <= s_axi_bvalid_next;

      for (int i = 0; i < WORD_WIDTH; i = i + 1) begin
         if (mem_wr_en & s_axi_wstrb[i]) begin
            mem[write_addr_valid][WORD_SIZE*i +: WORD_SIZE] <= s_axi_wdata[WORD_SIZE*i +: WORD_SIZE];
         end
      end

      if (rst) begin
         write_state_reg <= WRITE_STATE_IDLE;

         s_axi_awready_reg <= 1'b0;
         s_axi_wready_reg <= 1'b0;
         s_axi_bvalid_reg <= 1'b0;
      end
   end

   always_comb begin
      read_state_next = READ_STATE_IDLE;

      mem_rd_en = 1'b0;

      s_axi_rid_next = s_axi_rid_reg;
      s_axi_rlast_next = s_axi_rlast_reg;
      s_axi_rvalid_next = s_axi_rvalid_reg && !(s_axi_rready || (PIPELINE_OUTPUT && !s_axi_rvalid_pipe_reg));

      read_id_next = read_id_reg;
      read_addr_next = read_addr_reg;
      read_first_addr_next = read_first_addr_reg;
      read_last_addr_next = read_last_addr_reg;
      read_wrap_addr_next = read_wrap_addr_reg;
      read_count_next = read_count_reg;
      read_size_next = read_size_reg;
      read_burst_next = read_burst_reg;

      s_axi_arready_next = 1'b0;

      unique case (read_state_reg)
        READ_STATE_IDLE: begin
           s_axi_arready_next = 1'b1;

           if (s_axi_arready && s_axi_arvalid) begin
              read_id_next = s_axi_arid;
              read_addr_next = s_axi_araddr;
              read_first_addr_next = s_axi_araddr;
              read_count_next = s_axi_arlen;
              read_size_next = s_axi_arsize < axsize_t'($clog2(STRB_WIDTH)) ? s_axi_arsize : axsize_t'($clog2(STRB_WIDTH));
              read_burst_next = s_axi_arburst;
              read_wrap_addr_next = s_axi_araddr & ~(((ADDR_WIDTH'(1) << read_size_next) * (ADDR_WIDTH'(s_axi_arlen) + ADDR_WIDTH'(1))) - ADDR_WIDTH'(1));
              read_last_addr_next = read_wrap_addr_next + ((ADDR_WIDTH'(1) << read_size_next) * (ADDR_WIDTH'(s_axi_arlen) + ADDR_WIDTH'(1))) - ADDR_WIDTH'(1);

              s_axi_arready_next = 1'b0;
              read_state_next = READ_STATE_BURST;
           end else begin
              read_state_next = READ_STATE_IDLE;
           end
        end
        READ_STATE_BURST: begin
           if (s_axi_rready || (PIPELINE_OUTPUT && !s_axi_rvalid_pipe_reg) || !s_axi_rvalid_reg) begin
              mem_rd_en = 1'b1;
              s_axi_rvalid_next = 1'b1;
              s_axi_rid_next = read_id_reg;
              s_axi_rlast_next = read_count_reg == 0;
              if (read_burst_reg != 2'b00) begin
                 read_addr_next = read_addr_reg + (1 << read_size_reg);
              end
              if (read_burst_reg == 2'b10) begin
                 if (read_addr_next > read_last_addr_reg) begin
                    read_addr_next = read_wrap_addr_reg;
                 end
              end
              read_count_next = read_count_reg - 1;
              if (read_count_reg > 0) begin
                 read_state_next = READ_STATE_BURST;
              end else begin
                 s_axi_arready_next = 1'b1;
                 read_state_next = READ_STATE_IDLE;
              end
           end else begin
              read_state_next = READ_STATE_BURST;
           end
        end
        default: ;
      endcase
   end

   always @(posedge clk) begin
      read_state_reg <= read_state_next;

      read_id_reg <= read_id_next;
      read_addr_reg <= read_addr_next;
      read_first_addr_reg <= read_first_addr_next;
      read_last_addr_reg <= read_last_addr_next;
      read_wrap_addr_reg <= read_wrap_addr_next;
      read_count_reg <= read_count_next;
      read_size_reg <= read_size_next;
      read_burst_reg <= read_burst_next;

      s_axi_arready_reg <= s_axi_arready_next;
      s_axi_rid_reg <= s_axi_rid_next;
      s_axi_rlast_reg <= s_axi_rlast_next;
      s_axi_rvalid_reg <= s_axi_rvalid_next;

      if (mem_rd_en) begin
         s_axi_rdata_reg <= mem[read_addr_valid];
      end

      if (!s_axi_rvalid_pipe_reg || s_axi_rready) begin
         s_axi_rid_pipe_reg <= s_axi_rid_reg;
         s_axi_rdata_pipe_reg <= s_axi_rdata_reg;
         s_axi_rlast_pipe_reg <= s_axi_rlast_reg;
         s_axi_rvalid_pipe_reg <= s_axi_rvalid_reg;
      end

      if (rst) begin
         read_state_reg <= READ_STATE_IDLE;

         s_axi_arready_reg <= 1'b0;
         s_axi_rvalid_reg <= 1'b0;
         s_axi_rvalid_pipe_reg <= 1'b0;
      end
   end

endmodule // axi_ram
