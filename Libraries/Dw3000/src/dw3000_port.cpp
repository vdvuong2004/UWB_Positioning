/*
 * Platform layer of the DW3000 driver: SPI, reset, delays and IRQ stubs.
 *
 * Based on the Makerfabs Arduino port (Emin Eminof, 2021).
 */

#include "dw3000_port.h"
#include "SPI.h"

#if !defined(ARDUINO_ARCH_STM32)
#define DW_SPI_PORT SPI
#else
static void stm32_bitbang_pins(void);
#endif

/* Clocked out while reading. */
#define JUNK 0x00

static uint8_t _ss;
static uint8_t _rst = 0xff;
static uint8_t _irq;
static bool _irq_enabled = true;

#if !defined(ARDUINO_ARCH_STM32)
/* DW3000 SPI must stay at or below 7 MHz until the PLL is locked. */
static const SPISettings _fastSPI = SPISettings(7000000L, MSBFIRST, SPI_MODE0);
static const SPISettings _slowSPI = SPISettings(2000000L, MSBFIRST, SPI_MODE0);
static const SPISettings *_currentSPI = &_slowSPI;
#endif

void deca_sleep(uint8_t time_ms)
{
  delay(time_ms);
}

void deca_usleep(uint8_t time_us)
{
  delayMicroseconds(time_us);
}

void Sleep(uint32_t d)
{
  delay(d);
}

void spiBegin(uint8_t irq, uint8_t rst)
{
  delay(5);

  pinMode(irq, INPUT);
  pinMode(rst, INPUT_PULLUP);
  pinMode(DW3000_PIN_CS, OUTPUT);
  digitalWrite(DW3000_PIN_CS, HIGH);

  /* CS stays a GPIO so the host does not release it between SPI bytes. */
#if defined(ARDUINO_ARCH_ESP32)
  _currentSPI = &_slowSPI;
  DW_SPI_PORT.begin(DW3000_PIN_SCK, DW3000_PIN_MISO, DW3000_PIN_MOSI, -1);
#elif defined(ARDUINO_ARCH_STM32)
  /* Hardware SPI reads the device ID, but every other command byte arrives
   * at the DW3000 as 0x00. Drive SCK/MOSI from GPIO instead. */
  stm32_bitbang_pins();
#else
  _currentSPI = &_slowSPI;
  DW_SPI_PORT.begin();
#endif

  _rst = rst;
  _irq = irq;
}

void reset(void)
{
  if (_rst == 0xff) {
    return;
  }
  /* RSTn is open-drain: drive it low, then release it. Never drive it high. */
  pinMode(_rst, OUTPUT);
  digitalWrite(_rst, LOW);
  delay(2);
  pinMode(_rst, INPUT_PULLUP);
  delay(20);
}

#if defined(ARDUINO_ARCH_STM32) && defined(STM32F103xE)
/* The core's Generic F103RCTx clock runs from the internal HSI. Use the
 * custom board's 8 MHz crystal (Y1 on PD0/PD1) for 72 MHz and an exact UART
 * baud. Falls back to HSI 64 MHz (48 MHz with USB) if the crystal does not start. */
bool dw3000_clock_from_hse = false;

extern "C" void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  dw3000_clock_from_hse = (HAL_RCC_OscConfig(&RCC_OscInitStruct) == HAL_OK);
  if (!dw3000_clock_from_hse) {
    RCC_OscInitStruct = {};
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSEState = RCC_HSE_OFF;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
#if defined(USBCON)
    /* 48 MHz: the only HSI clock USB can run from (PLL / 1). */
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
#else
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
#endif
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
      Error_Handler();
    }
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }

#if defined(USBCON)
  /* USB needs exactly 48 MHz: 72 / 1.5 from the crystal, 48 / 1 from HSI. */
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {};
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = dw3000_clock_from_hse ? RCC_USBCLKSOURCE_PLL_DIV1_5 : RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
    Error_Handler();
  }
#endif
}
#endif

#if defined(ARDUINO_ARCH_STM32)
static GPIO_TypeDef *_bb_sck_port;
static GPIO_TypeDef *_bb_mosi_port;
static GPIO_TypeDef *_bb_miso_port;
static uint32_t _bb_sck_bit;
static uint32_t _bb_mosi_bit;
static uint32_t _bb_miso_bit;

/* Half-clock wait. 40 gives ~10 us at 72 MHz (~165 us per byte). Override with
 * -DDW3000_STM32_BB_LOOPS=n to trade SPI speed against margin. */
#ifndef DW3000_STM32_BB_LOOPS
#define DW3000_STM32_BB_LOOPS 40
#endif

static void stm32_bb_wait(void)
{
  /* delayMicroseconds(1) returns immediately on this core, so MOSI was still
   * changing on the same edge as SCK. */
  for (volatile uint16_t n = 0; n < DW3000_STM32_BB_LOOPS; n++) {
    __NOP();
  }
}

static void stm32_bitbang_pins(void)
{
  /* SPI1 stays in AF on PA5/PA7 and holds MOSI low, so every header except
   * the all-zero DEV_ID read is received as 0x00. */
#ifdef SPI1
  RCC->APB2ENR |= (1U << 12);
  SPI1->CR1 &= ~(1U << 6);
#endif
  pinMode(DW3000_PIN_SCK, OUTPUT);
  pinMode(DW3000_PIN_MOSI, OUTPUT);
  pinMode(DW3000_PIN_MISO, INPUT);
  digitalWrite(DW3000_PIN_SCK, LOW);
  digitalWrite(DW3000_PIN_MOSI, LOW);
  _bb_sck_port = digitalPinToPort(DW3000_PIN_SCK);
  _bb_mosi_port = digitalPinToPort(DW3000_PIN_MOSI);
  _bb_miso_port = digitalPinToPort(DW3000_PIN_MISO);
  _bb_sck_bit = digitalPinToBitMask(DW3000_PIN_SCK);
  _bb_mosi_bit = digitalPinToBitMask(DW3000_PIN_MOSI);
  _bb_miso_bit = digitalPinToBitMask(DW3000_PIN_MISO);
  if (_bb_sck_port == NULL || _bb_mosi_port == NULL || _bb_miso_port == NULL) {
    UART_puts((char *)"SPI pin fail\r\n");
  }
}

static uint8_t stm32_bb_transfer(uint8_t tx)
{
  uint8_t rx = 0;
  if (_bb_sck_port == NULL || _bb_mosi_port == NULL || _bb_miso_port == NULL) {
    return 0;
  }
  for (int8_t bit = 7; bit >= 0; bit--) {
    if ((tx >> bit) & 1U) {
      _bb_mosi_port->BSRR = _bb_mosi_bit;
    } else {
      _bb_mosi_port->BSRR = _bb_mosi_bit << 16;
    }
    stm32_bb_wait();
    _bb_sck_port->BSRR = _bb_sck_bit;
    stm32_bb_wait();
    rx <<= 1;
    if (_bb_miso_port->IDR & _bb_miso_bit) {
      rx |= 1U;
    }
    _bb_sck_port->BSRR = _bb_sck_bit << 16;
  }
  return rx;
}
#endif

static uint8_t dw_xfer(uint8_t data)
{
#if defined(ARDUINO_ARCH_STM32)
  return stm32_bb_transfer(data);
#else
  return DW_SPI_PORT.transfer(data);
#endif
}

static void dw_spi_open(void)
{
#if !defined(ARDUINO_ARCH_STM32)
  DW_SPI_PORT.beginTransaction(*_currentSPI);
#endif
  digitalWrite(_ss, LOW);
#if defined(ARDUINO_ARCH_STM32)
  stm32_bb_wait();
#endif
}

static void dw_spi_close(void)
{
  delayMicroseconds(5);
  digitalWrite(_ss, HIGH);
#if !defined(ARDUINO_ARCH_STM32)
  DW_SPI_PORT.endTransaction();
#endif
}

void spiSelect(uint8_t ss)
{
  _ss = ss;
  pinMode(_ss, OUTPUT);
  digitalWrite(_ss, HIGH);
  reset();
}

void spiSetFastRate(void)
{
  /* The STM32 bit-bang rate is fixed by DW3000_STM32_BB_LOOPS. */
#if !defined(ARDUINO_ARCH_STM32)
  _currentSPI = &_fastSPI;
#endif
}

int readfromspi(uint16_t headerLength, uint8_t *headerBuffer, uint16_t readLength, uint8_t *readBuffer)
{
  dw_spi_open();
  for (uint16_t i = 0; i < headerLength; i++) {
    dw_xfer(headerBuffer[i]);
  }
  for (uint16_t i = 0; i < readLength; i++) {
    readBuffer[i] = dw_xfer(JUNK);
  }
  dw_spi_close();
  return 0;
}

int writetospi(uint16_t headerLength, uint8_t *headerBuffer, uint16_t bodyLength, uint8_t *bodyBuffer)
{
  dw_spi_open();
  for (uint16_t i = 0; i < headerLength; i++) {
    dw_xfer(headerBuffer[i]);
  }
  for (uint16_t i = 0; i < bodyLength; i++) {
    dw_xfer(bodyBuffer[i]);
  }
  dw_spi_close();
  return 0;
}

/* Wakes the DW3000 from sleep by holding CS low. */
void wakeup_device_with_io(void)
{
  digitalWrite(_ss, LOW);
  delay(2);
  digitalWrite(_ss, HIGH);
}

uint32_t port_GetEXT_IRQStatus(void)
{
  return _irq_enabled ? 1UL : 0UL;
}

uint32_t port_CheckEXT_IRQ(void)
{
  return digitalRead(_irq) ? 1UL : 0UL;
}

void port_DisableEXT_IRQ(void)
{
  _irq_enabled = false;
}

void port_EnableEXT_IRQ(void)
{
  _irq_enabled = true;
}

static port_dwic_isr_t port_dwic_isr = NULL;

void port_set_dwic_isr(port_dwic_isr_t dwic_isr)
{
  portDISABLE_INTERRUPTS();
  port_dwic_isr = dwic_isr;
  portENABLE_INTERRUPTS();
}
