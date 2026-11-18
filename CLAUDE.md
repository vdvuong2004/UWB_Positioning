# CLAUDE.md

This file guides Claude Code when it works in this repository.

## What this is

UWB indoor positioning. A tag (custom STM32F103RCT6 board + DWM3000) runs single-sided
TWR against several anchors (DWM3000 or DWM1000) in a fixed 25 ms frame, compensates (`scale·d − bias`)
median-5 + IIR + 2 cm dead band filters each range on the tag, and prints `DIST <id> <filtered m> m raw <raw m>` (USB only).
`Tools/uwb_tracker` (Python) reads those lines, runs an EKF and draws a 2D map.
The user-facing overview is `README.md` (Vietnamese). Talk to the user in Vietnamese.

Layout:
- `Libraries/Dw3000` — Arduino library `DW3000`: Decawave DW3000 C0 driver (API 04.00.00) + platform layer.
  Examples: `uwb_tag`, `uwb_anchor_dw3000`, `debug_board_test`, `debug_radio_link`.
- `Libraries/Dw1000` — Arduino library `DW1000`: low-level part of thotro/arduino-dw1000 + STM32 bit-bang SPI.
  Example: `uwb_anchor_dw1000`.
- `Tools/uwb_tracker` — `uwb_tracker.py` (EKF + matplotlib), `anchors.json` (anchor positions by ID).
- `install_libraries.bat` — junctions both libraries into `Documents\Arduino\libraries`.

## Build and flash

No build system or tests. Check a change by compiling every affected example:

```
arduino-cli compile --fqbn STMicroelectronics:stm32:GenF1:pnum=GENERIC_F103RCTX,usb=CDCgen Libraries/Dw3000/examples/uwb_tag
arduino-cli compile --fqbn esp32:esp32:esp32   Libraries/Dw3000/examples/uwb_anchor_dw3000
arduino-cli compile --fqbn esp32:esp32:esp32c3 Libraries/Dw1000/examples/uwb_anchor_dw1000
```

arduino-cli ships with the IDE at `E:\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe`.
The libraries resolve through the junctions in `Documents\Arduino\libraries`; if a build behaves like
old code, check that those are still junctions to this repo. Custom boards upload over SWD (ST-Link,
`upload -p COMx` works, the port is only used to find the board); ESP32 boards over their COM port.
Custom boards log on their own USB CDC port; open it with DTR set or the sketch waits/loses the banner.

## Protocol invariants (tag and both anchor sketches must agree)

- Radio: channel 5, PLEN 128, PAC8, code 9, **SFD 0 (IEEE 8-symbol)** so the DWM1000 decodes it, 6.8 Mb/s, STS off.
- Poll (12 B incl. CRC): `41 88 seq CA DE dst src E0`. Resp (20 B incl. CRC): `41 88 seq CA DE dst src E1 poll_rx_ts(4) resp_tx_ts(4)`.
  Addresses 16-bit LE; anchors 1.., tag 100. The anchor copies seq; the tag checks seq, dst, src and func.
- Anchors reply exactly `REPLY_DELAY_US` = 3000 after the poll RX timestamp. The custom board needs 1.9 ms
  of prep with `DW3000_STM32_BB_LOOPS` 2 (DW1000 anchor: DX_TIME = poll RX ts + delay, not `setDelay()`, which counts
  from now). Tag: RX opens 2000 uus after TX, timeout 4000 uus; a miss is retried at once (`max_tries`, cmd `r`).
- The DW3000 response buffer is 20 bytes: 19 clips the TX timestamp. The DW1000 library's
  `getDataLength()` excludes the 2 CRC bytes (poll = 10).
- TX power adapts (tag v2.5 / DW3000 anchor v2.2): the tag steps each anchor through `PWR_LADDER` {0x48, 0x80, 0xC0, 0xFD}
  (up on a miss, wrapping to the bottom; down after 20 replies above -75 dBm), start 0x48 for DWM3000, 0xC0 for DWM1000.
  `0xC0` saturates a nearby DWM3000 so it never decodes. The DW3000 anchor replies at 0xFD when the poll is below
  -85 dBm, else at its low `TX_POWER` (0x48 STM32, 0x80 WROOM). RX level = DW3000 manual 4.7.2 (IP_DIAG_1/12, DGC 0x30060).
- Antenna delay: DW3000 16385, DW1000 16436 (not calibrated per module yet; `bias_m` in `anchors.json` covers it).

## Platform details that are easy to break

- **DW3000 pins are compiled into the library** (`src/dw3000_board_config.h`); a sketch `#define` does not reach the `.cpp`.
- **STM32 uses bit-banged SPI** in both libraries. STM32 hardware SPI read the DEV_ID but corrupted every other
  command byte. DW3000: `DW3000_STM32_BB_LOOPS` (2 on the custom board, error-free tested).
  DW1000: MISO is sampled before the rising edge; GPIO registers are written directly and the wait drops to
  `DW1000_STM32_BB_FAST_LOOPS` 0 after PLL lock, else the reply prep (now ~2.1 ms) misses the 3 ms delay.
- Bit-banged SPI sometimes returns glitch reads: read `SYS_STATUS` twice and drop reads with the top bits set or
  the `0xCA03` pattern (`read_status()` in the DW3000 sketches). Keep that filter.
- STM32 F103xE: `SystemClock_Config` in `dw3000_port.cpp` runs from the 8 MHz crystal (72 MHz, USB 48 MHz) and
  falls back to HSI. `dw3000_clock_from_hse` reports which.
- Console: `test_run_info()`. STM32: USART1 PA9/PA10, mirrored to USB CDC. ESP32: Serial, or `printf` when USB CDC
  On Boot is off (ESP32-C3).
- RSTn is open-drain: drive low, release to `INPUT_PULLUP`, never drive high.
- The DW1000 anchor must re-arm RX after an RX error (`attachReceiveFailedHandler` + watchdog); without it one bad
  frame from other UWB traffic stops the receiver for good. Also attach the timeout handler (an SFD timeout stops RX too)
  and keep the watchdog at 10 ms: at 50 ms its ~0.7 silent RX stops/s cost ~3% of the polls.
  Never call `DW1000.handleInterrupt()` on a timer without a pending event: it reads SYS_STATUS once and then
  clears every bit, so a frame finishing in between is wiped and RX stays idle (v2.4 lost ~9% of the polls this
  way; v2.5 checks the status first, `rearm` went 1005 -> 0, anchor 2 replies 99.8%).
- ESP32-WROOM GPIO 6–11 are flash pins.

## Hardware history (for debugging)

- A module that runs "sometimes" was every time a supply problem: the open GND pin of header J7/J8
  on board A (module GND floated 1–2 V), contact resistance on board C, or a wiring mistake on the test stations.
  Measure module GND vs. board GND and the module's 3V3 before blaming code.
- DWM3000 VDD readings from the chip (~4.65 V) are an OTP reference error, not the real supply.

## Conventions

- Sketches: `#define APP_NAME "… vX.Y"`, bump the version when you change a sketch. Fatal errors go to `halt("…")`.
  Init order: `UART_init` → `spiBegin` → `spiSelect` → poll IDLE_RC/DEV_ID → `dwt_initialise` → `dwt_configure` →
  `dwt_configuretxrf` → `spiSetFastRate`.
- Comments are short and say why (often a hardware quirk). Keep platform code in `dw3000_port.cpp`,
  `dw3000_uart.cpp`, `dw3000_mutex.cpp`, `dw3000_board_config.h`, behind `#if defined(ARDUINO_ARCH_*)`.
- The vendor files (`dw3000_device_api.*`, `dw3000_regs.h`, `dw3000_vals.h`) stay unmodified unless a fix belongs there.
- Commits: no Co-Authored-By / AI attribution lines. The user pushes directly to `main`.
