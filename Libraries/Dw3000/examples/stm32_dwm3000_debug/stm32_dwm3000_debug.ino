#include "dw3000.h"
#include <SPI.h>

/*
 * STM32 + DWM3000 link test. Flash this with ST-LINK. No COM port is required.
 *
 * Wiring: SCK PA5, MISO PA6, MOSI PA7, CS PA4, IRQ PB1, RST PB0.
 * Onboard LED is PC13, active low.
 *
 * After boot the LED blinks once, then repeats one of these:
 *   slow on/off     DEV ID OK (DECA0302 or DECA0312)
 *   1 fast blink    ID is 00000000  (MISO stuck low, or the module has no power)
 *   2 fast blinks   ID is FFFFFFFF  (MISO stuck high, or SCK/CS is not reaching the module)
 *   3 fast blinks   some other ID   (SPI moves, but it is not a DW3000)
 *
 * Debug log is USART1 at 115200:
 *   STM32 PA9  TX -> USB-UART RX
 *   STM32 PA10 RX -> USB-UART TX
 *   GND to GND
 * Open the USB-UART COM port in the serial monitor. The ST-LINK port is not used.
 */

#if !defined(ARDUINO_ARCH_STM32)
#error "Select the STM32 board. This sketch is the STM32 to DWM3000 link test."
#endif

static const uint8_t LED_PIN = PC13;

static void ledOff(void)
{
  digitalWrite(LED_PIN, HIGH);
}

static void ledOn(void)
{
  digitalWrite(LED_PIN, LOW);
}

static void blinkTimes(int count)
{
  for (int i = 0; i < count; i++) {
    ledOn();
    delay(120);
    ledOff();
    delay(180);
  }
}

static Uart BoardUart(USART1);

static void logUart(const char *text)
{
  BoardUart.println(text);
}

static uint32_t rawDevId(void)
{
  uint8_t byte0;
  uint8_t byte1;
  uint8_t byte2;
  uint8_t byte3;

  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
  digitalWrite(DW3000_PIN_CS, LOW);
  SPI.transfer(0x00);
  byte0 = SPI.transfer(0x00);
  byte1 = SPI.transfer(0x00);
  byte2 = SPI.transfer(0x00);
  byte3 = SPI.transfer(0x00);
  digitalWrite(DW3000_PIN_CS, HIGH);
  SPI.endTransaction();

  return (uint32_t)byte0
      | ((uint32_t)byte1 << 8)
      | ((uint32_t)byte2 << 16)
      | ((uint32_t)byte3 << 24);
}

static bool idIsDw3000(uint32_t dev_id)
{
  return (dev_id == DWT_C0_DEV_ID) || (dev_id == DWT_C0_PDOA_DEV_ID);
}

void setup()
{
  pinMode(LED_PIN, OUTPUT);
  ledOff();

  BoardUart.setTx(PA9);
  BoardUart.setRx(PA10);
  BoardUart.begin(115200);
  delay(200);

  blinkTimes(1);
  logUart("STM32 DWM3000 debug");

  spiBegin(DW3000_PIN_IRQ, DW3000_PIN_RST);
  spiSelect(DW3000_PIN_CS);
  delay(5);
}

void loop()
{
  uint32_t raw_id = rawDevId();
  uint32_t lib_id = dwt_readdevid();
  char line[48];

  snprintf(line, sizeof(line), "RAW ID: %08lX", (unsigned long)raw_id);
  logUart(line);
  snprintf(line, sizeof(line), "LIB ID: %08lX", (unsigned long)lib_id);
  logUart(line);

  if (idIsDw3000(raw_id) || idIsDw3000(lib_id)) {
    logUart("DEV ID OK");
    ledOn();
    delay(1500);
    ledOff();
    delay(400);
    return;
  }

  if ((raw_id == 0x00000000UL) && (lib_id == 0x00000000UL)) {
    logUart("DEV ID 00000000");
    blinkTimes(1);
  } else if ((raw_id == 0xFFFFFFFFUL) && (lib_id == 0xFFFFFFFFUL)) {
    logUart("DEV ID FFFFFFFF");
    blinkTimes(2);
  } else {
    logUart("DEV ID UNEXPECTED");
    blinkTimes(3);
  }

  delay(900);
}
