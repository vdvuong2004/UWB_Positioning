/*
 * Board defaults for the DW3000 Arduino port.
 *
 * ESP32-C3 SuperMini pin configuration
 */

#ifndef DW3000_BOARD_CONFIG_H_
#define DW3000_BOARD_CONFIG_H_

#if defined(ARDUINO_ARCH_NRF52)

#ifndef DW3000_PIN_RST
#define DW3000_PIN_RST 14
#endif

#ifndef DW3000_PIN_IRQ
#define DW3000_PIN_IRQ 12
#endif

#ifndef DW3000_PIN_CS
#define DW3000_PIN_CS 7
#endif

#ifndef DW3000_PIN_SCK
#define DW3000_PIN_SCK 5
#endif

#ifndef DW3000_PIN_MOSI
#define DW3000_PIN_MOSI 9
#endif

#ifndef DW3000_PIN_MISO
#define DW3000_PIN_MISO 10
#endif

#elif defined(ARDUINO_ARCH_STM32)

// ============================================================
// STM32 (SPI1) + DWM3000
// Blue Pill / generic F103 defaults. Change these to match your wiring.
//   DWM3000 SCK  -> PA5
//   DWM3000 MISO -> PA6
//   DWM3000 MOSI -> PA7
//   DWM3000 CS   -> PA4
//   DWM3000 IRQ  -> PB1
//   DWM3000 RST  -> PB0
// ============================================================

#ifndef DW3000_PIN_RST
#define DW3000_PIN_RST PB0
#endif

#ifndef DW3000_PIN_IRQ
#define DW3000_PIN_IRQ PB1
#endif

#ifndef DW3000_PIN_CS
#define DW3000_PIN_CS PA4
#endif

#ifndef DW3000_PIN_SCK
#define DW3000_PIN_SCK PA5
#endif

#ifndef DW3000_PIN_MISO
#define DW3000_PIN_MISO PA6
#endif

#ifndef DW3000_PIN_MOSI
#define DW3000_PIN_MOSI PA7
#endif

#elif defined(ARDUINO_ESP32C3_DEV) || defined(CONFIG_IDF_TARGET_ESP32C3)

/* ESP32-C3 SuperMini. These pins are compiled into the library, not only the sketch. */
#ifndef DW3000_PIN_RST
#define DW3000_PIN_RST 2
#endif
#ifndef DW3000_PIN_IRQ
#define DW3000_PIN_IRQ 3
#endif
#ifndef DW3000_PIN_CS
#define DW3000_PIN_CS 7
#endif
#ifndef DW3000_PIN_SCK
#define DW3000_PIN_SCK 4
#endif
#ifndef DW3000_PIN_MISO
#define DW3000_PIN_MISO 5
#endif
#ifndef DW3000_PIN_MOSI
#define DW3000_PIN_MOSI 6
#endif

#elif defined(ARDUINO_ARCH_ESP32)

/* ESP32-WROOM-32 Dev Module. GPIO 6-11 are the module flash and must not be used. */
#ifndef DW3000_PIN_RST
#define DW3000_PIN_RST 4
#endif
#ifndef DW3000_PIN_IRQ
#define DW3000_PIN_IRQ 22
#endif
#ifndef DW3000_PIN_CS
#define DW3000_PIN_CS 21
#endif
#ifndef DW3000_PIN_SCK
#define DW3000_PIN_SCK 18
#endif
#ifndef DW3000_PIN_MISO
#define DW3000_PIN_MISO 19
#endif
#ifndef DW3000_PIN_MOSI
#define DW3000_PIN_MOSI 23
#endif

#else

/* Fallback, same as the ESP32-C3 SuperMini. */
#ifndef DW3000_PIN_RST
#define DW3000_PIN_RST 2
#endif
#ifndef DW3000_PIN_IRQ
#define DW3000_PIN_IRQ 3
#endif
#ifndef DW3000_PIN_CS
#define DW3000_PIN_CS 7
#endif
#ifndef DW3000_PIN_SCK
#define DW3000_PIN_SCK 4
#endif
#ifndef DW3000_PIN_MISO
#define DW3000_PIN_MISO 5
#endif
#ifndef DW3000_PIN_MOSI
#define DW3000_PIN_MOSI 6
#endif

#endif

#endif /* DW3000_BOARD_CONFIG_H_ */