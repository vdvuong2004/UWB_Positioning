/*
 * Console used by every sketch (test_run_info).
 * STM32: USART1 PA9/PA10, mirrored to the board's USB CDC port when enabled.
 * ESP32: Serial, or printf when USB CDC On Boot is off.
 *
 * Based on the Makerfabs DW3000 Arduino port.
 */

#include "dw3000_uart.h"

#if defined(ARDUINO_ARCH_STM32)
/* Core 3.0 leaves Serial1 unlinked unless that UART is enabled in the
 * board menu. Uart is the concrete port; TX stays PA9, RX stays PA10. */
static Uart DwUart(USART1);
#if defined(USBCON) && defined(USBD_USE_CDC)
/* Tools > USB support > CDC: mirror the console to the board's own USB port. */
#define DW_USB_CONSOLE 1
#endif
#endif

void UART_init(void)
{
#if defined(ARDUINO_ARCH_STM32)
    DwUart.setTx(PA9);
    DwUart.setRx(PA10);
    DwUart.begin(115200);
#if defined(DW_USB_CONSOLE)
    SerialUSB.begin(115200);
#endif
#else
    Serial.begin(115200);
#if defined(ARDUINO_USB_CDC_ON_BOOT) && ARDUINO_USB_CDC_ON_BOOT
    uint32_t start = millis();
    while (!Serial && (millis() - start < 2000)) {
        delay(10);
    }
#endif
#endif
}

static void console_putc(char data)
{
#if defined(ARDUINO_ARCH_STM32)
    DwUart.write((uint8_t)data);
#if defined(DW_USB_CONSOLE)
    SerialUSB.write((uint8_t)data);
#endif
#elif defined(ARDUINO_ARCH_ESP32) && !(defined(ARDUINO_USB_CDC_ON_BOOT) && ARDUINO_USB_CDC_ON_BOOT)
    printf("%c", data);
#else
    Serial.write((uint8_t)data);
#endif
}

static void console_puts(const char *s)
{
#if defined(ARDUINO_ARCH_STM32)
    DwUart.print(s);
#if defined(DW_USB_CONSOLE)
    SerialUSB.print(s);
#endif
#elif defined(ARDUINO_ARCH_ESP32) && !(defined(ARDUINO_USB_CDC_ON_BOOT) && ARDUINO_USB_CDC_ON_BOOT)
    printf("%s", s);
#else
    Serial.print(s);
#endif
}

void UART_putc(char data)
{
    console_putc(data);
}

void UART_puts(char* s)
{
    if (s != NULL) {
        console_puts(s);
    }
}

void test_run_info(unsigned char *s)
{
    UART_puts((char *)s);
    UART_puts((char *)"\r\n");
#if defined(ARDUINO_ARCH_STM32)
    DwUart.flush();
#elif !defined(ARDUINO_ARCH_ESP32) || (defined(ARDUINO_USB_CDC_ON_BOOT) && ARDUINO_USB_CDC_ON_BOOT)
    Serial.flush();
#endif
}