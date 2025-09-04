# TMDSCNCD28P55X ↔ DRV8301 SPI — Interrupts with Command Queue + Callbacks (no DriverLib)

Pure *Peripheral Header Files* implementation for F28P55x. Features:
- SPIA **RX FIFO ISR** + **CPU Timer0** periodic trigger (100 ms default).
- A **ring-buffer command queue** for DRV8301 reads/writes.
- Optional **per-transaction callbacks** (executed in ISR context).
- Live mirror of **STATUS1** in `gStatus1` and LED blink on FAULT.

## Files
- `include/board_regs_it.h` — config macros and board init prototypes
- `include/drv8301.h` — DRV8301 register bitfields + queue/callback API
- `src/board_regs_it.c` — clock/PIE, GPIO mux, SPIA+FIFO setup, Timer0 mapping
- `src/drv8301_it.c` — ISR + queue implementation (core logic)
- `src/main.c` — example usage

## Build (CCS)
1. New *empty* project for **TMS320F28P55x** (no DriverLib).
2. Add C2000Ware *Peripheral Header Files* include path (so `F28x_Project.h` resolves).
3. Add these `src/` and `include/` files to your project.
4. Ensure your clock defines match (example assumes **CPU = 200 MHz**, **LSPCLK = 50 MHz**).
5. Wire: GPIO16 SIMO, GPIO17 SOMI, GPIO18 SCLK, GPIO19 nCS (manual) or SPISTE if you set `USE_MANUAL_CS` to 0.
