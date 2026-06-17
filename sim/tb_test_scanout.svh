// -*- mode: verilog; mode: font-lock; indent-tabs-mode: nil -*-
// vi: set et ts=3 sw=3 sts=3:
//
// Copyright 2026 Steffen Persvold
// SPDX-License-Identifier: Apache-2.0
//
// ========================================================================
// File        : tb_test_scanout.svh
// Author      : Steffen Persvold
// Created     : June 17, 2026
// ========================================================================
// Description : Scanout testcase. Owns the cfg_* register-bus BFM and the
//               framebuffer-setup tasks/palette, programs a 640x480x32
//               framebuffer, and runs the scanout engine for a while.
//               Selected with +TESTCASE=scanout (the default).
// ========================================================================
//

   // -------------------------------------------------------------------
   // Scanout register-bus (cfg_*) BFM + framebuffer-setup helpers
   // -------------------------------------------------------------------

   typedef struct packed {
      logic [11:0] adr;
      logic        we;
      logic [ 3:0] be;
      logic [31:0] d;
   } cfg_req_t;

   cfg_req_t cfg_requests[$];
   logic           pending;

   always @(posedge clk_sys) begin
      if (rst_sys) begin
         cfg_req <= 1'b0;
         pending <= 1'b0;
      end
      else begin
         cfg_req <= 1'b0;
         cfg_adr <= 'x;
         cfg_we  <= 'x;
         cfg_be  <= 'x;
         cfg_d   <= 'x;

         if (cfg_ack) pending <= 1'b0;

         if (cfg_requests.size() > 0 && (!pending || cfg_ack)) begin
            automatic cfg_req_t req;
            req = cfg_requests.pop_front();

            cfg_req <= 1'b1;
            cfg_adr <= req.adr[11:2];
            cfg_we  <= req.we;
            cfg_be  <= req.be;
            cfg_d   <= req.d;
            pending  <= 1'b1;
         end
      end
   end

   localparam FB_SYNC_HOR_HIGH_ACT   =  1;      /* horizontal sync high active  */
   localparam FB_SYNC_VERT_HIGH_ACT  =  2;      /* vertical sync high active    */
   localparam FB_SYNC_EXT            =  4;      /* external sync                */
   localparam FB_SYNC_COMP_HIGH_ACT  =  8;      /* composite sync high active   */
   localparam FB_SYNC_BROADCAST      =  16;     /* broadcast video timings      */
                                                /* vtotal = 144d/288n/576i => PAL  */
                                                /* vtotal = 121d/242n/484i => NTSC */
   localparam FB_SYNC_ON_GREEN       =  32;     /* sync on green */

   localparam FB_VMODE_NONINTERLACED =  0;      /* non interlaced */
   localparam FB_VMODE_INTERLACED    =  1;      /* interlaced   */
   localparam FB_VMODE_DOUBLE        =  2;      /* double scan */
   localparam FB_VMODE_ODD_FLD_FIRST =  4;      /* interlaced: top line first */
   localparam FB_VMODE_MASK          =  255;

   typedef struct packed {
      logic [31:0] xres;                /* visible resolution           */
      logic [31:0] yres;
      logic [31:0] xres_virtual;        /* virtual resolution           */
      logic [31:0] yres_virtual;
      logic [31:0] bits_per_pixel;      /* guess what                   */
      logic        grayscale;
      logic [31:0] left_margin;         /* time from sync to picture    */
      logic [31:0] right_margin;        /* time from picture to sync    */
      logic [31:0] upper_margin;        /* time from sync to picture    */
      logic [31:0] lower_margin;
      logic [31:0] hsync_len;           /* length of horizontal sync    */
      logic [31:0] vsync_len;           /* length of vertical sync      */
      logic [31:0] sync;                /* see FB_SYNC_*                */
      logic [31:0] vmode;               /* see FB_VMODE_*               */
   } fbmode_t;

   localparam OCFB_CTRL    = 'h0;
   localparam OCFB_STAT    = 'h4;
   localparam OCFB_HTIM    = 'h8;
   localparam OCFB_VTIM    = 'hc;
   localparam OCFB_HVLEN   = 'h10;
   localparam OCFB_VBARA   = 'h14;
   localparam OCFB_VSIZ    = 'h18;
   localparam OCFB_PITCH   = 'h20;
   localparam OCFB_PALETTE = 'h800;

   localparam [31:0] OCFB_CTRL_VEN = 32'h00000001; /* Video Enable */
   localparam [31:0] OCFB_CTRL_HIE = 32'h00000002; /* HSync Interrupt Enable */
   localparam [31:0] OCFB_CTRL_PC  = 32'h00000800; /* 8-bit Pseudo Color Enable*/
   localparam [31:0] OCFB_CTRL_CD8 = 32'h00000000; /* Color Depth 8 */
   localparam [31:0] OCFB_CTRL_CD16= 32'h00000200; /* Color Depth 16 */
   localparam [31:0] OCFB_CTRL_CD24= 32'h00000400; /* Color Depth 24 */
   localparam [31:0] OCFB_CTRL_CD32= 32'h00000600; /* Color Depth 32 */
   localparam [31:0] OCFB_CTRL_VBL1= 32'h00000000; /* Burst Length 1 */
   localparam [31:0] OCFB_CTRL_VBL2= 32'h00000080; /* Burst Length 2 */
   localparam [31:0] OCFB_CTRL_VBL4= 32'h00000100; /* Burst Length 4 */
   localparam [31:0] OCFB_CTRL_VBL8= 32'h00000180; /* Burst Length 8 */
   localparam [31:0] OCFB_CTRL_HSL = 32'h00001000; /* HSync level */
   localparam [31:0] OCFB_CTRL_VSL = 32'h00002000; /* VSync level */
   localparam [31:0] OCFB_CTRL_DBL = 32'h00010000; /* Doublescan mode */

   task ocfb_writereg(input logic [11:0] offset, input logic [31:0] data);
      begin
         cfg_requests.push_back({offset, 1'b1, 4'hf, data});
         wait (cfg_requests.size() == 0);
      end
   endtask // ocfb_writereg

   task ocfb_setupfb(input logic [31:0] fb_phys, input fbmode_t fbmode);

      logic [31:0] ctrl;
      logic [31:0] hlen;
      logic [31:0] vlen;
      logic [31:0] pitch;
      logic [31:0] fb_size;

      begin
         /* Disable display */
         ocfb_writereg(OCFB_CTRL, 0);

         /* Register framebuffer address */
         ocfb_writereg(OCFB_VBARA, fb_phys);

         /* Register framebuffer size */
         fb_size = fbmode.xres_virtual * fbmode.yres_virtual * (fbmode.bits_per_pixel/8);
	 ocfb_writereg(OCFB_VSIZ, fb_size);

         /* Horizontal timings */
         ocfb_writereg(OCFB_HTIM, (fbmode.hsync_len - 1) << 24 |
                       (fbmode.left_margin - 1) << 16 | (fbmode.xres - 1));

         /* Vertical timings */
         ocfb_writereg(OCFB_VTIM, (fbmode.vsync_len - 1) << 24 |
                       (fbmode.upper_margin - 1) << 16 | (fbmode.yres - 1));

         /* Total length of frame */
         hlen = fbmode.left_margin  + fbmode.right_margin + fbmode.hsync_len + fbmode.xres;
         vlen = fbmode.upper_margin + fbmode.lower_margin + fbmode.vsync_len + fbmode.yres;
         ocfb_writereg(OCFB_HVLEN, (hlen - 1) << 16 | (vlen - 1));

	 /* Horizontal pitch */
	 pitch = ((fbmode.xres_virtual * fbmode.bits_per_pixel/8) -
                  (fbmode.xres * fbmode.bits_per_pixel/8));
	 ocfb_writereg(OCFB_PITCH, pitch);

         ctrl = OCFB_CTRL_CD8;
         case (fbmode.bits_per_pixel)
           8 : if (!fbmode.grayscale) ctrl |= OCFB_CTRL_PC;  /* enable palette */
           16: ctrl |= OCFB_CTRL_CD16;
           24: ctrl |= OCFB_CTRL_CD24;
           32: ctrl |= OCFB_CTRL_CD32;
           default: $error("no bpp specified");
         endcase

	 if ((fbmode.sync & FB_SYNC_HOR_HIGH_ACT) == 0)
	   ctrl |= OCFB_CTRL_HSL;
	 if ((fbmode.sync & FB_SYNC_VERT_HIGH_ACT) == 0)
	   ctrl |= OCFB_CTRL_VSL;
         if ((fbmode.vmode & FB_VMODE_DOUBLE) != 0)
           ctrl |= OCFB_CTRL_DBL;

         /* maximum (8) VBL (video memory burst length) */
         ctrl |= OCFB_CTRL_VBL8;

         /* Enable output */
         ocfb_writereg(OCFB_CTRL, (OCFB_CTRL_VEN | ctrl));
      end
   endtask // ocfb_setupfb

   typedef struct packed {
      logic [7:0] transp;
      logic [7:0] red;
      logic [7:0] green;
      logic [7:0] blue;
   } pal_t;

   task ocfb_setcolreg(input integer regno, pal_t pal);
      begin

	 ocfb_writereg(OCFB_PALETTE + 12'(regno<<2),
                       (32'(pal.red) << 16) | (32'(pal.green) << 8) | 32'(pal.blue));
      end
   endtask // ocfb_setcolreg


   pal_t palette[256] = '{32'hFF000000, 32'hFF800000, 32'hFF008000, 32'hFF808000, 32'hFF000080, 32'hFF800080, 32'hFF008080, 32'hFFc0c0c0,
                          32'hFFc0dcc0, 32'hFFa6caf0, 32'hFF2a3faa, 32'hFF2a3fff, 32'hFF2a5f00, 32'hFF2a5f55, 32'hFF2a5faa, 32'hFF2a5fff,
                          32'hFF2a7f00, 32'hFF2a7f55, 32'hFF2a7faa, 32'hFF2a7fff, 32'hFF2a9f00, 32'hFF2a9f55, 32'hFF2a9faa, 32'hFF2a9fff,
                          32'hFF2abf00, 32'hFF2abf55, 32'hFF2abfaa, 32'hFF2abfff, 32'hFF2adf00, 32'hFF2adf55, 32'hFF2adfaa, 32'hFF2adfff,
                          32'hFF2aff00, 32'hFF2aff55, 32'hFF2affaa, 32'hFF2affff, 32'hFF550000, 32'hFF550055, 32'hFF5500aa, 32'hFF5500ff,
                          32'hFF551f00, 32'hFF551f55, 32'hFF551faa, 32'hFF551fff, 32'hFF553f00, 32'hFF553f55, 32'hFF553faa, 32'hFF553fff,
                          32'hFF555f00, 32'hFF555f55, 32'hFF555faa, 32'hFF555fff, 32'hFF557f00, 32'hFF557f55, 32'hFF557faa, 32'hFF557fff,
                          32'hFF559f00, 32'hFF559f55, 32'hFF559faa, 32'hFF559fff, 32'hFF55bf00, 32'hFF55bf55, 32'hFF55bfaa, 32'hFF55bfff,
                          32'hFF55df00, 32'hFF55df55, 32'hFF55dfaa, 32'hFF55dfff, 32'hFF55ff00, 32'hFF55ff55, 32'hFF55ffaa, 32'hFF55ffff,
                          32'hFF7f0000, 32'hFF7f0055, 32'hFF7f00aa, 32'hFF7f00ff, 32'hFF7f1f00, 32'hFF7f1f55, 32'hFF7f1faa, 32'hFF7f1fff,
                          32'hFF7f3f00, 32'hFF7f3f55, 32'hFF7f3faa, 32'hFF7f3fff, 32'hFF7f5f00, 32'hFF7f5f55, 32'hFF7f5faa, 32'hFF7f5fff,
                          32'hFF7f7f00, 32'hFF7f7f55, 32'hFF7f7faa, 32'hFF7f7fff, 32'hFF7f9f00, 32'hFF7f9f55, 32'hFF7f9faa, 32'hFF7f9fff,
                          32'hFF7fbf00, 32'hFF7fbf55, 32'hFF7fbfaa, 32'hFF7fbfff, 32'hFF7fdf00, 32'hFF7fdf55, 32'hFF7fdfaa, 32'hFF7fdfff,
                          32'hFF7fff00, 32'hFF7fff55, 32'hFF7fffaa, 32'hFF7fffff, 32'hFFaa0000, 32'hFFaa0055, 32'hFFaa00aa, 32'hFFaa00ff,
                          32'hFFaa1f00, 32'hFFaa1f55, 32'hFFaa1faa, 32'hFFaa1fff, 32'hFFaa3f00, 32'hFFaa3f55, 32'hFFaa3faa, 32'hFFaa3fff,
                          32'hFFaa5f00, 32'hFFaa5f55, 32'hFFaa5faa, 32'hFFaa5fff, 32'hFFaa7f00, 32'hFFaa7f55, 32'hFFaa7faa, 32'hFFaa7fff,
                          32'hFFaa9f00, 32'hFFaa9f55, 32'hFFaa9faa, 32'hFFaa9fff, 32'hFFaabf00, 32'hFFaabf55, 32'hFFaabfaa, 32'hFFaabfff,
                          32'hFFaadf00, 32'hFFaadf55, 32'hFFaadfaa, 32'hFFaadfff, 32'hFFaaff00, 32'hFFaaff55, 32'hFFaaffaa, 32'hFFaaffff,
                          32'hFFd40000, 32'hFFd40055, 32'hFFd400aa, 32'hFFd400ff, 32'hFFd41f00, 32'hFFd41f55, 32'hFFd41faa, 32'hFFd41fff,
                          32'hFFd43f00, 32'hFFd43f55, 32'hFFd43faa, 32'hFFd43fff, 32'hFFd45f00, 32'hFFd45f55, 32'hFFd45faa, 32'hFFd45fff,
                          32'hFFd47f00, 32'hFFd47f55, 32'hFFd47faa, 32'hFFd47fff, 32'hFFd49f00, 32'hFFd49f55, 32'hFFd49faa, 32'hFFd49fff,
                          32'hFFd4bf00, 32'hFFd4bf55, 32'hFFd4bfaa, 32'hFFd4bfff, 32'hFFd4df00, 32'hFFd4df55, 32'hFFd4dfaa, 32'hFFd4dfff,
                          32'hFFd4ff00, 32'hFFd4ff55, 32'hFFd4ffaa, 32'hFFd4ffff, 32'hFFff0055, 32'hFFff00aa, 32'hFFff1f00, 32'hFFff1f55,
                          32'hFFff1faa, 32'hFFff1fff, 32'hFFff3f00, 32'hFFff3f55, 32'hFFff3faa, 32'hFFff3fff, 32'hFFff5f00, 32'hFFff5f55,
                          32'hFFff5faa, 32'hFFff5fff, 32'hFFff7f00, 32'hFFff7f55, 32'hFFff7faa, 32'hFFff7fff, 32'hFFff9f00, 32'hFFff9f55,
                          32'hFFff9faa, 32'hFFff9fff, 32'hFFffbf00, 32'hFFffbf55, 32'hFFffbfaa, 32'hFFffbfff, 32'hFFffdf00, 32'hFFffdf55,
                          32'hFFffdfaa, 32'hFFffdfff, 32'hFFffff55, 32'hFFffffaa, 32'hFFccccff, 32'hFFffccff, 32'hFF33ffff, 32'hFF66ffff,
                          32'hFF99ffff, 32'hFFccffff, 32'hFF007f00, 32'hFF007f55, 32'hFF007faa, 32'hFF007fff, 32'hFF009f00, 32'hFF009f55,
                          32'hFF009faa, 32'hFF009fff, 32'hFF00bf00, 32'hFF00bf55, 32'hFF00bfaa, 32'hFF00bfff, 32'hFF00df00, 32'hFF00df55,
                          32'hFF00dfaa, 32'hFF00dfff, 32'hFF00ff55, 32'hFF00ffaa, 32'hFF2a0000, 32'hFF2a0055, 32'hFF2a00aa, 32'hFF2a00ff,
                          32'hFF2a1f00, 32'hFF2a1f55, 32'hFF2a1faa, 32'hFF2a1fff, 32'hFF2a3f00, 32'hFF2a3f55, 32'hFFfffbf0, 32'hFFa0a0a4,
                          32'hFF808080, 32'hFFff0000, 32'hFF00ff00, 32'hFFffff00, 32'hFF0000ff, 32'hFFff00ff, 32'hFF00ffff, 32'hFFffffff
                         };

   task test_scanout;

      fbmode_t     fbmode;
      logic [19:0] vadr;
      realtime     t;

      begin
         fbmode.xres           = 640;
         fbmode.yres           = 480;
         fbmode.xres_virtual   = 640;
         fbmode.yres_virtual   = 480;
         fbmode.bits_per_pixel = 32;
         fbmode.left_margin    = 40;
         fbmode.right_margin   = 24;
         fbmode.upper_margin   = 32;
         fbmode.lower_margin   = 11;
         fbmode.hsync_len      = 96;
         fbmode.vsync_len      = 2;
         fbmode.sync           = 0;
         fbmode.vmode          = 0;

         #200;
         $display("%t INFO: Set up palette", $time);
         for (int i = 0; i<256; i++)
           ocfb_setcolreg(i, palette[i]);

         #200;
         $display("%t INFO: Initialize controller", $time);
         ocfb_setupfb(32'h0, fbmode);
         $display("%t INFO: Done", $time);

         #20ms;

         result = 1'b1;
      end
   endtask // test_scanout
