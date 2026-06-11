# vctrl — Video Controller

A small, synthesizable **video display controller** written in SystemVerilog. It
scans a framebuffer out of memory and drives an RGB/VGA-style display interface
with programmable timing, supporting 8/16/24/32 bits-per-pixel modes, an indexed
(pseudo-color) palette, and optional scan doubling.

## Features

- **Programmable display timing** — fully software-configurable horizontal and
  vertical sync/gate/length parameters, so resolution and refresh are not
  baked into the RTL.
- **Multiple pixel formats** — 8 bpp (grayscale or palette-indexed), 16 bpp
  (RGB565), 24 bpp (RGB888 packed), and 32 bpp (RGB0888).
- **256-entry color lookup table (CLUT)** for 8 bpp pseudo-color mode, with
  arbitration between CPU palette updates and the scanout read path.
- **Scan doubling** — line/frame doubling for low-resolution modes.
- **Clean clock-domain separation** — a system/memory clock (`clk_sys`) and an
  independent pixel clock (`clk_pix`), with an asynchronous line buffer and
  proper CDC primitives between them.
- **Register-compatible with the OpenCores VGA/LCD core** — the CSR map matches
  the layout driven by the mainline Linux `ocfb` framebuffer driver
  (`drivers/video/fbdev/ocfb.c`), so it can be driven by existing software.
- **Simple register/CSR interface** plus a split framebuffer read bus that can
  be attached to any memory subsystem.
- **Optional AXI4 read master** (`vctrl_axim`) that bridges the framebuffer
  read bus onto an AXI4 interconnect, including data-width conversion and burst
  prefetching.

## Architecture

```
            clk_sys domain                        │   clk_pix domain
                                                  │
  CSR/cfg ──► vctrl_regs ──► ctrl/timing/pitch ───┼──► vctrl_timing ──► sync/gate
                  │                               │          │
                  ▼                               │          ▼
  fb read bus ─► vctrl_cproc ─► RGB888 ─► vctrl_lbuff (async line buffer) ─► vctrl_fbuff ─► VGA out
                  │  ▲                            │
                  ▼  │ palette                    │
              vctrl_clut (CLUT RAM)               │
```

| Module          | Responsibility                                                        |
|-----------------|-----------------------------------------------------------------------|
| `vctrl_core`    | Top level; wires all sub-blocks together.                             |
| `vctrl_pkg`     | Shared constants, register-address map, and type definitions.         |
| `vctrl_regs`    | Control/status registers (CSRs) and interrupt generation.             |
| `vctrl_timing`  | Generates hsync/vsync/gate/frame/line from the timing registers.      |
| `vctrl_cproc`   | Color processor: converts framebuffer words to RGB888 per pixel format.|
| `vctrl_clut`    | 256-entry palette RAM with config/scanout arbitration.                |
| `vctrl_lbuff`   | Asynchronous line buffer crossing `clk_sys` → `clk_pix`.              |
| `vctrl_fbuff`   | Framebuffer datapath; assembles the final VGA output.                 |
| `vctrl_axim`    | Optional AXI4 read-master bridge for the framebuffer read bus.        |

The design fetches pixel data on `clk_sys` via a simple split read bus
(`fb_rdreq` / `fb_raddr` / `fb_rdack` / `fb_rdata` / `fb_rvalid`), converts it
to RGB888, and stages it in an asynchronous line buffer. The timing generator,
running on `clk_pix`, reads the line buffer and produces the display output and
sync signals.

## Register map

The CSR map is compatible with the [OpenCores VGA/LCD 2.0
core](https://opencores.org/projects/vga_lcd), which is what the mainline Linux
`ocfb` driver (`drivers/video/fbdev/ocfb.c`) programs — the register offsets and
`CTRL`/`STAT` bit fields match. The `PITCH` register (`0x20`) and the doublescan
control bit (`CTRL[16]`) are extensions beyond that core.

CSR accesses use a word address on `cfg_adr[11:2]`. Addresses with
`cfg_adr[11]` set are routed to the CLUT (256 entries × 24-bit RGB).

| Offset | Name    | Description                                          |
|--------|---------|------------------------------------------------------|
| `0x00` | `CTRL`  | Enable, interrupt enables, color depth, polarities, scan double. |
| `0x04` | `STAT`  | hsync/vsync interrupt status (write-1-to-clear).     |
| `0x08` | `HTIM`  | Horizontal timing: sync width, gate delay, gate width. |
| `0x0C` | `VTIM`  | Vertical timing: sync width, gate delay, gate width.   |
| `0x10` | `HVLEN` | Total horizontal / vertical line lengths.            |
| `0x14` | `VBAR`  | Video base address (framebuffer scanout base).       |
| `0x20` | `PITCH` | Horizontal line pitch (bytes).                       |
| `0x800+` | CLUT  | Palette entries (8 bpp pseudo-color mode).           |

See `rtl/vctrl_pkg.sv` for the exact register and bit-field layout.

## Repository layout

```
rtl/                 RTL source for the video controller
  rtl_files.f        Compile-order file list
  vctrl_*.sv         Controller modules
sub/common/          Reusable RTL primitives (git submodule)
  rtl/               BRAMs, FIFOs, CDC primitives, synchronizers, etc.
```

This repo uses a git submodule for shared primitives (`bram_sdp`, `bram_1rw`,
`cdc_tgl`, `synchronizer`, FIFOs, …). Clone with submodules:

```sh
git clone --recurse-submodules <repo-url>
# or, if already cloned:
git submodule update --init --recursive
```

## Simulation / build

The controller RTL compile order is given in `rtl/rtl_files.f`, and the common
primitives in `sub/common/rtl/filelist.f`. Point your simulator at both file
lists, e.g. with VCS:

```sh
vcs -sverilog -f sub/common/rtl/filelist.f -f rtl/rtl_files.f
```

or any other SystemVerilog-2012 capable tool (Verilator, Questa, Xcelium, …).

## License

Apache License 2.0. See [LICENSE](LICENSE).

Copyright © 2021–2026 Steffen Persvold.
