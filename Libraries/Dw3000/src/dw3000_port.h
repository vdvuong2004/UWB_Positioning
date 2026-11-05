/*
 * Platform layer of the DW3000 driver: SPI, reset, delays and IRQ stubs.
 *
 * Based on the Makerfabs Arduino port (Emin Eminof, 2021). Supports the custom
 * STM32F103RCT6 board (bit-banged SPI) and ESP32 / ESP32-C3 (hardware SPI).
 */

#ifndef PORT_H_
#define PORT_H_

#include "dw3000_board_config.h"
#include "dw3000.h"

#if !defined(portDISABLE_INTERRUPTS)
#define portDISABLE_INTERRUPTS() noInterrupts()
#endif
#if !defined(portENABLE_INTERRUPTS)
#define portENABLE_INTERRUPTS() interrupts()
#endif

#ifndef FALSE
#define FALSE 0
#endif
#ifndef TRUE
#define TRUE 1
#endif

/* Sets up the SPI pins (slow rate) and remembers the IRQ and RST pins. */
void spiBegin(uint8_t irq, uint8_t rst);
/* Selects the CS pin and pulses RSTn. dwt_initialise() does the register setup. */
void spiSelect(uint8_t ss);
/* Full SPI speed. Call only after dwt_initialise() and dwt_configure(). */
void spiSetFastRate(void);
/* Hardware reset through the open-drain RSTn pin. */
void reset(void);

#if defined(ARDUINO_ARCH_STM32) && defined(STM32F103xE)
/* True when SystemClock_Config() started the custom board's 8 MHz crystal. */
extern bool dw3000_clock_from_hse;
#endif

/* Called by the vendor driver. */
int writetospi(uint16_t headerLength, uint8_t *headerBuffer, uint16_t bodyLength, uint8_t *bodyBuffer);
int readfromspi(uint16_t headerLength, uint8_t *headerBuffer, uint16_t readLength, uint8_t *readBuffer);
void Sleep(uint32_t d);
void wakeup_device_with_io(void);

uint32_t port_GetEXT_IRQStatus(void);
uint32_t port_CheckEXT_IRQ(void);
void port_DisableEXT_IRQ(void);
void port_EnableEXT_IRQ(void);

/* DW IC IRQ handler type. The examples poll SYS_STATUS and do not use it. */
typedef void (*port_dwic_isr_t)(void);
void port_set_dwic_isr(port_dwic_isr_t isr);

#endif /* PORT_H_ */
