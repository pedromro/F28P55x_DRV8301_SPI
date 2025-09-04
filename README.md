# TMDSCNCD28P55X ↔ DRV8301 SPI — **Interrupt-Driven** (no DriverLib)

This version replaces polling with **interrupts**:
- **SPIA RX FIFO interrupt** captures incoming 16‑bit words.
- **CPU Timer0** periodically kicks off a DRV8301 **Status1** read (every 100 ms).
- A tiny state machine in the RX ISR assembles the 2‑frame read and updates `gStatus1`.
- LED (GPIO31) toggles from the ISR when `FAULT` is set.

Still pure **Peripheral Header Files** (no DriverLib).

## Wiring (unchanged)
- GPIO16 → SPISIMOA, GPIO17 → SPISOMIA, GPIO18 → SPICLKA
- GPIO19 → **manual** nCS (change `USE_MANUAL_CS` to 0 to use SPISTE).

## Build
- Same as the previous register-level project; ensure `F28x_Project.h` resolves
  and `LSPCLK_HZ` matches your clock tree.
