# DW1000 (UWB_Positioning fork)

Low-level DW1000 driver from [thotro/arduino-dw1000](https://github.com/thotro/arduino-dw1000)
(Apache 2.0, see `LICENSE.md`), trimmed and adapted for this project.

Changes from upstream:

- STM32 (STM32duino core): SPI is bit-banged on the pins given to `DW1000.setSpiPins()`.
  The STM32 hardware SPI was unreliable on the custom board. MISO is sampled before the
  rising edge, because before the PLL locks the DW1000 shifts the next bit out soon after it.
- ESP32: `setSpiPins()` selects the SPI pins.
- Removed: the high-level `DW1000Ranging` / `DW1000Device` / `DW1000Mac` framework, the
  upstream examples, the adapter board files and the generated docs. Ranging uses this
  project's own protocol instead (see the top-level README).

Example: `examples/uwb_anchor_dw1000`.
