# CLAUDE.md

This file guides Claude Code (claude.ai/code) when it works in this repository.

## What this is

An Arduino library (`DW3000`, v0.1) that wraps the Decawave/Qorvo DW3000 C0 device driver (API 04.00.00, `src/dw3000_version.h`) for UWB radio and two-way ranging. It started as the Makerfabs Arduino port and has been changed to run on several MCUs:

- **ESP32-C3 SuperMini**: the default and fallback pin set.
- **ESP32-WROOM-32** (`ARDUINO_ARCH_ESP32`)
- **STM32F103** (Blue Pill / F103C6 / F103RC, `ARDUINO_ARCH_STM32`, STM32duino core)
- **nRF52** (`ARDUINO_ARCH_NRF52`)

There is no build system, test suite or git history. A change is checked by compiling an example sketch in the Arduino IDE or with `arduino-cli`, for example:

```
arduino-cli compile --fqbn esp32:esp32:esp32c3 --library . examples/ex_00a_reading_dev_id
arduino-cli compile --fqbn STMicroelectronics:stm32:GenF1 --library . examples/ex_dwm3000_peer_resp
```

On real hardware, run `ex_00a_reading_dev_id` or `stm32_dwm3000_debug` first to prove the SPI link works. The expected device ID is `DECA0302` or `DECA0312` (`DWT_C0_DEV_ID` / `DWT_C0_PDOA_DEV_ID`).

## Source layout (`src/`)

Sketches include only `dw3000.h`. It pulls in uart, port, device_api and shared_functions.

| File | Role |
|---|---|
| `dw3000_device_api.*`, `dw3000_regs.h`, `dw3000_vals.h`, `dw3000_types.h` | Vendor Decawave driver (`dwt_*` functions, register IDs and masks). It is large and mostly unmodified, so don't change it unless a fix truly belongs there. |
| `dw3000_port.*` | **Platform layer.** SPI open/close/transfer, `readfromspi`/`writetospi` (called by the driver), reset, SPI speed, IRQ-enable stubs and `port_set_dwic_isr`. Also holds DW1000-era helpers (`readBytes`, `writeBytes`, `softReset`, `manageLDE` and others). Most of these are legacy, and the DW3000 examples don't use them. |
| `dw3000_board_config.h` | Per-architecture `DW3000_PIN_{RST,IRQ,CS,SCK,MISO,MOSI}` defaults inside `#ifndef` guards. |
| `dw3000_uart.*` | `UART_init`, `UART_puts`, `test_run_info`: the console every sketch uses. |
| `dw3000_mutex.cpp` | `decamutexon`/`decamutexoff`: `portENTER_CRITICAL` on ESP32 and nRF52, `noInterrupts()` elsewhere. |
| `dw3000_shared_functions.*` | Timestamp helpers (`get_rx_timestamp_u64`, `resp_msg_get_ts`/`set_ts`, `final_msg_*`) and STS/delay helpers. |
| `dw3000_config_options.*`, `dw3000_shared_defines.h` | Preset `dwt_config_t` values and constants (`UUS_TO_DWT_TIME`, `SPEED_OF_LIGHT`, and others). |
| `dw3000_mac_802_15_4.*` | 802.15.4 MAC header and AES helpers, used only by the `ex_06e`/`ex_06f` AES examples. |

All platform-specific code lives in `dw3000_port.cpp`, `dw3000_uart.cpp`, `dw3000_mutex.cpp` and `dw3000_board_config.h`. Keep it there and branch with the existing `#if defined(ARDUINO_ARCH_*)` pattern.

## Platform details that are easy to break

- **The pins are compiled into the library.** `spiBegin` uses `DW3000_PIN_*` from `dw3000_board_config.h` directly. A `#define` in a sketch does not reach the library's `.cpp` files, so changing the wiring means editing `dw3000_board_config.h` or passing `-D` build flags. Sketches copy the macros into `PIN_RST`/`PIN_IRQ`/`PIN_SS`.
- **CS is a plain GPIO**, never the SPI peripheral's hardware SS. On ESP32, `SPI.begin(sck, miso, mosi, -1)` is called for this reason.
- **SPI speed:** `spiBegin` starts at 2 MHz (`_slowSPI`). Sketches call `spiSetFastRate()` (7 MHz, the DW3000 limit before PLL lock) only after `dwt_initialise` and `dwt_configure`. Don't raise `_fastSPI` above 7 MHz.
- **STM32 uses bit-banged SPI** (`stm32_bitbang_pins` and `stm32_bb_transfer`, about 10 µs per half clock). STM32 hardware SPI1 could read the device ID, but every other command byte arrived as 0x00. The SPI1 peripheral is disabled and SCK/MOSI are driven through BSRR. As a result, `spiSetFastRate()` does nothing on STM32, and STM32 responders need long reply delays. The half-clock wait is `DW3000_STM32_BB_LOOPS` (default 40, ~228 µs per byte including CS overhead). The custom F103RC board sets it to 2 in `dw3000_board_config.h` (~22 µs per byte). That value was tested error-free on two boards, before and after RX.
- **The STM32 console is USART1 on PA9/PA10** (`Uart DwUart(USART1)`), not `Serial`, which is the USB/ST-LINK port on that core. With Tools > USB support: CDC (`USBCON` + `USBD_USE_CDC`), `test_run_info` also mirrors to `SerialUSB`, and the F103xE `SystemClock_Config` sets the 48 MHz USB clock (HSI fallback drops to 48 MHz). On ESP32 without USB-CDC, output goes through `printf`.
- **RST is open-drain.** `reset()` drives the pin low for 2 ms, then releases it to `INPUT_PULLUP`. Never drive it high.
- **ESP32-WROOM:** GPIO 6–11 are the flash pins. Don't assign them.
- The IRQ functions (`port_GetEXT_IRQStatus` and others) only flip a flag. The examples poll `SYS_STATUS` and don't use interrupts.

## Examples (`examples/`)

- `ex_00a` … `ex_06f`: ports of the Decawave reference examples (simple TX/RX, TX sleep, SS-TWR with and without STS, and AES).
- `Makerfabs_*`: the original Makerfabs ranging and multi-anchor/multi-tag demos.
- `stm32_custom_pin_test`: bring-up test for the custom F103RCT6 board (pin levels, bit-banged and driver DEV_ID, init, configure, one TX), logged over the board USB CDC port. Needs Tools > USB support: CDC.
- `stm32_dwm3000_debug`: an SPI link test for STM32 that reports through LED blink codes on PC13 and USART1.
- `ex_stm32_alive`: a minimal STM32 blink and UART sanity check.
- `ex_dwm1000_*`: these use the separate **`DW1000`** library (arduino-dw1000), not this one, so they need that library installed.

### Multi-board ranging setup

A single-sided TWR setup in which one STM32 "logger" ranges against three responders in turn:

| Sketch | Board | Poll func code / ID | Response | Reply delay |
|---|---|---|---|---|
| `ex_dwm3000_dwm1000_init` (initiator/logger), `ex_dwm3000_range_fast` (no pause, rate counter) | STM32F103C8T6 + DWM3000 | sends all three | — | — |
| `ex_custom_board_init` (same initiator, USB CDC log, reply stats, `PEER_*` switches) | custom STM32F103RCT6 + DWM3000 | sends all three | — | — |
| `ex_dwm3000_peer_resp` | STM32F103RCT6 + DWM3000 | `0xE2` `'DW3A'` | `0xE3` `'A3WD'` | 2.5 ms on F103xB/xC/xE, otherwise 30 ms |
| `ex_custom_board_resp` (USB CDC log, stats) | custom STM32F103RCT6 + DWM3000 | `0xE2` `'DW3A'` | `0xE3` `'A3WD'` | 3 ms (reply prep measured 1.9 ms with `DW3000_STM32_BB_LOOPS` 2; it needs 20 ms at the default 40) |
| `ex_dwm3000_wroom_resp` | ESP32-WROOM-32 + DW3000 | `0xE4` `'WR32'` | `0xE5` `'23RW'` | 2.5 ms |
| `ex_dwm1000_range_resp` | ESP32-C3 + DWM1000 | `0xE0` `'WAVE'` | `0xE1` `'VEWA'` | about 1.5 ms |

With two custom boards, one runs `ex_custom_board_init` and the other `ex_custom_board_resp` (peer `'3000'`). Run only one `'DW3A'` responder at a time: `ex_custom_board_resp` and `ex_dwm3000_peer_resp` answer the same poll.

Invariants shared by these sketches. If you change one side, change the other.
- Radio: channel 5, PLEN 128, PAC8, preamble code 9, **SFD = 0 (IEEE 8-symbol)** so that the DWM1000 can decode it, 6.8 Mb/s, STS off.
- Frame: a 10-byte common header (`0x41 0x88 seq 0xCA 0xDE` + 4 ID chars + function code). The sequence number is at index 2 and is zeroed before `memcmp`. The response carries `poll_rx_ts` at index 10 and `resp_tx_ts` at index 14, then 2 CRC bytes. The response buffer is 20 bytes, because a 19-byte buffer clips the TX timestamp.
- Antenna delays are 16385. TX power to the DWM3000 peers is `0x48484848`. `0xC0C0C0C0` saturates the other DWM3000, which then never replies. The DWM1000 uses `0xC0C0C0C0`.
- Every responder's reply delay must land inside the initiator's `*_RX_DLY_UUS` / `*_RX_TIMEOUT_UUS` window for that peer.
- The initiator reads `SYS_STATUS` twice and discards reads with the top bits set or with the `0xCA03` pattern. These are glitch reads from the bit-banged SPI. Keep that filter.

## Conventions

- Sketch style: `#define APP_NAME "… vX.Y"` printed with `test_run_info`. Bump the version when you change a sketch. Fatal errors go to `halt("…")`. Init order: `UART_init` → `spiBegin` → `spiSelect` → poll `dwt_checkidlerc` → `dwt_initialise(DWT_DW_INIT)` → `dwt_configure` → `dwt_configuretxrf` → `spiSetFastRate`.
- Comments are short and state why something is done, often a hardware quirk. Follow that style.
