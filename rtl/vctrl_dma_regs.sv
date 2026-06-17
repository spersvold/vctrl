// -*- mode: verilog; mode: font-lock; indent-tabs-mode: nil -*-
// vi: set et ts=3 sw=3 sts=3:
//
// Copyright 2026 Steffen Persvold
// SPDX-License-Identifier: Apache-2.0
//
// ========================================================================
// File        : vctrl_dma_regs.sv
// Author      : Steffen Persvold
// Created     : June 17, 2026
// ========================================================================
// Description : DMA engine CSRs.
//
//   Slave on the cmd_* register bus (same request/ack protocol as the
//   core's cfg_* bus). Holds the PIO descriptor registers, emits a one-
//   cycle desc_go pulse on a doorbell write, and exposes status / fence /
//   interrupt back to software. The datapath FSM consumes `desc` on
//   desc_go and drives the status/fence/event inputs.
// ========================================================================
//

module vctrl_dma_regs import vctrl_pkg::*;
  (
   input  logic         clk_sys,
   input  logic         rst_sys,

   // ----------------------------------------------------------------------
   // Register bus slave (cmd_*)
   // ----------------------------------------------------------------------
   input  logic         cmd_req,
   input  logic [11: 2] cmd_adr,
   input  logic         cmd_we,
   input  logic [ 3: 0] cmd_be,
   input  logic [31: 0] cmd_d,
   output logic [31: 0] cmd_q,
   output logic         cmd_ack,

   // ----------------------------------------------------------------------
   // Control plane to the datapath / FSM
   // ----------------------------------------------------------------------
   output logic         dma_enable,   // CTRL.enable level
   output logic         dma_soft_rst, // CTRL.reset, one-cycle pulse
   output dma_desc_t    desc,         // current PIO descriptor
   output logic         desc_go,      // one-cycle pulse: execute `desc`

   // ----------------------------------------------------------------------
   // Status plane from the datapath / FSM
   // ----------------------------------------------------------------------
   input  logic         busy,         // a command is executing
   input  logic         idle,         // engine idle, no in-flight beats
   input  logic         dma_error,    // sticky error level
   input  logic [ 7: 0] state,        // FSM state (debug)
   input  logic [31: 0] fence_seqno,  // last completed seqno
   input  logic [31: 0] err_info,     // error info (resp / fault addr)
   input  logic         done_set,     // one-cycle: command completed
   input  logic         err_set,      // one-cycle: command faulted

   // ----------------------------------------------------------------------
   // Interrupt
   // ----------------------------------------------------------------------
   output logic         irq
   );

   wire [REG_ADR_HIBIT : 2] reg_adr = cmd_adr[REG_ADR_HIBIT : 2];

   wire reg_acc  = cmd_req;
   wire reg_wacc = cmd_req & cmd_we;

   // Control / interrupt
   logic [ 1:0] irq_status;  // {error, done} (write-1-to-clear)
   logic [ 1:0] irq_en;

   logic [31:0] reg_dato;

   // -------------------------------------------------------------------
   // Single-cycle acknowledge (mirrors vctrl_regs)
   // -------------------------------------------------------------------
   always_ff @(posedge clk_sys)
     if (rst_sys)
       cmd_ack <= 1'b0;
     else
       cmd_ack <= reg_acc & ~cmd_ack;

   // -------------------------------------------------------------------
   // Descriptor + control registers
   // -------------------------------------------------------------------
   always_ff @(posedge clk_sys)
     if (rst_sys)
       begin
          desc         <= '0;
          dma_enable   <= 1'b0;
          dma_soft_rst <= 1'b0;
          desc_go      <= 1'b0;
          irq_en       <= '0;
       end
     else
       begin
          // default the one-cycle pulses
          dma_soft_rst <= 1'b0;
          desc_go      <= 1'b0;

          if (reg_wacc)
            unique case (reg_adr)
              DMA_CTRL_ADR     : begin
                 dma_enable   <= cmd_d[DMA_CTRL_ENABLE];
                 dma_soft_rst <= cmd_d[DMA_CTRL_RESET];
              end
              DMA_IRQEN_ADR    : irq_en         <= cmd_d[1:0];
              DMA_SRC_ADR      : desc.src_addr  <= cmd_d;
              DMA_DST_ADR      : desc.dst_addr  <= cmd_d;
              DMA_SRCPITCH_ADR : desc.src_pitch <= cmd_d;
              DMA_DSTPITCH_ADR : desc.dst_pitch <= cmd_d;
              DMA_WIDTH_ADR    : desc.width     <= cmd_d;
              DMA_HEIGHT_ADR   : desc.height    <= cmd_d;
              DMA_OP_ADR       : desc.opflags   <= {16'h0, cmd_d[15:0]};
              DMA_DOORBELL_ADR : begin
                 desc.seqno <= cmd_d;  // fence value to report on completion
                 desc_go    <= 1'b1;   // kick the FSM with the current descriptor
              end
              default:;
            endcase
       end

   // -------------------------------------------------------------------
   // Interrupt status: set by FSM events, write-1-to-clear by software
   // (mirrors the vctrl_regs hint/vint idiom)
   // -------------------------------------------------------------------
   always_ff @(posedge clk_sys)
     if (rst_sys)
       irq_status <= '0;
     else if (reg_wacc & (reg_adr == DMA_IRQ_ADR))
       begin
          irq_status[DMA_IRQ_DONE]  <= done_set | (irq_status[DMA_IRQ_DONE]  & ~cmd_d[DMA_IRQ_DONE]);
          irq_status[DMA_IRQ_ERROR] <= err_set  | (irq_status[DMA_IRQ_ERROR] & ~cmd_d[DMA_IRQ_ERROR]);
       end
     else
       begin
          irq_status[DMA_IRQ_DONE]  <= irq_status[DMA_IRQ_DONE]  | done_set;
          irq_status[DMA_IRQ_ERROR] <= irq_status[DMA_IRQ_ERROR] | err_set;
       end

   always_ff @(posedge clk_sys)
     irq <= |(irq_status & irq_en);

   // -------------------------------------------------------------------
   // Read mux
   // -------------------------------------------------------------------
   always_comb
     unique casez (reg_adr)
       DMA_ID_ADR       : reg_dato = DMA_IDENT;
       DMA_CTRL_ADR     : reg_dato = {31'h0, dma_enable};
       DMA_STATUS_ADR   : reg_dato = {20'h0, state, 1'b0, dma_error, idle, busy};
       DMA_IRQ_ADR      : reg_dato = {30'h0, irq_status};
       DMA_IRQEN_ADR    : reg_dato = {30'h0, irq_en};
       DMA_SRC_ADR      : reg_dato = desc.src_addr;
       DMA_DST_ADR      : reg_dato = desc.dst_addr;
       DMA_SRCPITCH_ADR : reg_dato = desc.src_pitch;
       DMA_DSTPITCH_ADR : reg_dato = desc.dst_pitch;
       DMA_WIDTH_ADR    : reg_dato = desc.width;
       DMA_HEIGHT_ADR   : reg_dato = desc.height;
       DMA_OP_ADR       : reg_dato = desc.opflags;
       DMA_FENCE_ADR    : reg_dato = fence_seqno;
       DMA_ERR_ADR      : reg_dato = err_info;
       default          : reg_dato = 32'h0000_0000;
     endcase

   always_ff @(posedge clk_sys)
     if (reg_acc)
       cmd_q <= reg_dato;

endmodule // vctrl_dma_regs
