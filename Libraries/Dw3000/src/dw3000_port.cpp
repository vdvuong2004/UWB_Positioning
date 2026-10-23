/*
 * port.c
 *
 * Created: 9/10/2021 1:20:05 PM
 *  Author: Emim Eminof
 */ 

#include "dw3000_port.h"
#include "SPI.h"

#if !defined(ARDUINO_ARCH_STM32)
#define DW_SPI_PORT SPI
#else
static void stm32_bitbang_pins(void);
#endif

static void dw_spi_open(void);
static void dw_spi_header(const uint8_t *header, uint16_t headerLength);
static uint8_t dw_spi_byte(uint8_t data);
static void dw_spi_close(void);

uint8_t _ss;
uint8_t _rst;
uint8_t _irq;
static bool _irq_enabled = true;

#ifdef ESP8266
  // default ESP8266 frequency is 80 Mhz, thus divide by 4 is 20 MHz
  const SPISettings _fastSPI = SPISettings(8000000L, MSBFIRST, SPI_MODE0);
#else
  // DW3000 SPI must stay at or below 7 MHz until the PLL is locked.
  SPISettings _fastSPI = SPISettings(7000000L, MSBFIRST, SPI_MODE0);
#endif
const SPISettings _slowSPI = SPISettings(2000000L, MSBFIRST, SPI_MODE0);
const SPISettings* _currentSPI = &_fastSPI;

boolean _debounceClockEnabled = false;

/* SPI configs. */
/*const SPISettings _fastSPI;
const SPISettings _slowSPI;
const SPISettings* _currentSPI;*/

  /* register caches. */
byte _syscfg[LEN_SYS_CFG];
byte _sysctrl[LEN_SYS_CTRL];
byte _sysstatus[LEN_SYS_STATUS];
byte _txfctrl[LEN_TX_FCTRL];
byte _sysmask[LEN_SYS_MASK];
byte _chanctrl[LEN_CHAN_CTRL];

uint8_t _deviceMode;

/* device status monitoring */
byte _vmeas3v3;
byte _tmeas23C;
  
/* PAN and short address. */
byte _networkAndAddress[LEN_PANADR];

void enableDebounceClock() {
    byte pmscctrl0[LEN_PMSC_CTRL0];
    memset(pmscctrl0, 0, LEN_PMSC_CTRL0);
    readBytes(PMSC, PMSC_CTRL0_SUB, pmscctrl0, LEN_PMSC_CTRL0);
    setBit(pmscctrl0, LEN_PMSC_CTRL0, GPDCE_BIT, 1);
    setBit(pmscctrl0, LEN_PMSC_CTRL0, KHZCLKEN_BIT, 1);
    writeBytes(PMSC, PMSC_CTRL0_SUB, pmscctrl0, LEN_PMSC_CTRL0);
    _debounceClockEnabled = true;
}

void sleepms(uint32_t x)
{
  delay(x);
}

int sleepus(uint32_t x)
{
  delayMicroseconds(x);
  return 0;
}

void deca_sleep(uint8_t time_ms) // wrapper for decawave sleep function
{
  sleepms(time_ms);
}

void deca_usleep(uint8_t time_us) // wrapper for decawave sleep function
{
  sleepus(time_us);
}


void spiBegin(uint8_t irq, uint8_t rst)
{
    delay(5);

    pinMode(irq, INPUT);
    pinMode(rst, INPUT_PULLUP);
    pinMode(DW3000_PIN_CS, OUTPUT);
    digitalWrite(DW3000_PIN_CS, HIGH);

    // Use the slow rate until dwt_initialise() has read the device ID.
    _currentSPI = &_slowSPI;

    // CS stays a GPIO so the host does not release it between SPI bytes.
#if defined(ARDUINO_ARCH_ESP32)
    DW_SPI_PORT.begin(
        DW3000_PIN_SCK,
        DW3000_PIN_MISO,
        DW3000_PIN_MOSI,
        -1
    );
#elif defined(ARDUINO_ARCH_STM32)
    /* Hardware SPI reads the device ID, but every other command byte arrives
     * at the DW3000 as 0x00. Drive SCK/MOSI from GPIO instead. */
    pinMode(DW3000_PIN_CS, OUTPUT);
    digitalWrite(DW3000_PIN_CS, HIGH);
    stm32_bitbang_pins();
#else
    DW_SPI_PORT.begin();
#endif

    _rst = rst;
    _irq = irq;
}

void reselect(uint8_t ss) {
  _ss = ss;
  pinMode(_ss, OUTPUT);
  digitalWrite(_ss, HIGH);
}

void readBytes(byte cmd, uint16_t offset, byte data[], uint16_t n) {
  byte header[3];
  uint8_t headerLen = 1;
  uint16_t i = 0;
  
  // build SPI header
  if(offset == NO_SUB) {
    header[0] = READ | cmd;
  } else {
    header[0] = READ_SUB | cmd;
    if(offset < 128) {
      header[1] = (byte)offset;
      headerLen++;
    } else {
      header[1] = RW_SUB_EXT | (byte)offset;
      header[2] = (byte)(offset >> 7);
      headerLen += 2;
    }
  }
  dw_spi_open();
  dw_spi_header(header, headerLen);
  for(i = 0; i < n; i++) {
    data[i] = dw_spi_byte(JUNK); // read values
  }
  dw_spi_close();
}

// always 4 bytes
// TODO why always 4 bytes? can be different, see p. 58 table 10 otp memory map
void readBytesOTP(uint16_t address, byte data[]) {
  byte addressBytes[LEN_OTP_ADDR];
  
  // p60 - 6.3.3 Reading a value from OTP memory
  // bytes of address
  addressBytes[0] = (address & 0xFF);
  addressBytes[1] = ((address >> 8) & 0xFF);
  // set address
  writeBytes(OTP_IF, OTP_ADDR_SUB, addressBytes, LEN_OTP_ADDR);
  // switch into read mode
  writeByte(OTP_IF, OTP_CTRL_SUB, 0x03); // OTPRDEN | OTPREAD
  writeByte(OTP_IF, OTP_CTRL_SUB, 0x01); // OTPRDEN
  // read value/block - 4 bytes
  readBytes(OTP_IF, OTP_RDAT_SUB, data, LEN_OTP_RDAT);
  // end read mode
  writeByte(OTP_IF, OTP_CTRL_SUB, 0x00);
}

// Helper to set a single register
void writeByte(byte cmd, uint16_t offset, byte data) {
  writeBytes(cmd, offset, &data, 1);
}

/*
 * Write bytes to the DW1000. Single bytes can be written to registers via sub-addressing.
 * @param cmd
 *    The register address (see Chapter 7 in the DW1000 user manual).
 * @param offset
 *    The offset to select register sub-parts for writing, or 0x00 to disable
 *    sub-adressing.
 * @param data
 *    The data array to be written.
 * @param data_size
 *    The number of bytes to be written (take care not to go out of bounds of
 *    the register).
 */
// TODO offset really bigger than byte?
void writeBytes(byte cmd, uint16_t offset, byte data[], uint16_t data_size) {
  byte header[3];
  uint8_t  headerLen = 1;
  uint16_t  i = 0;
  
  // TODO proper error handling: address out of bounds
  // build SPI header
  if(offset == NO_SUB) {
    header[0] = WRITE | cmd;
  } else {
    header[0] = WRITE_SUB | cmd;
    if(offset < 128) {
      header[1] = (byte)offset;
      headerLen++;
    } else {
      header[1] = RW_SUB_EXT | (byte)offset;
      header[2] = (byte)(offset >> 7);
      headerLen += 2;
    }
  }
  dw_spi_open();
  dw_spi_header(header, headerLen);
  for(i = 0; i < data_size; i++) {
    dw_spi_byte(data[i]); // write values
  }
  dw_spi_close();
}

void enableClock(byte clock) {
  byte pmscctrl0[LEN_PMSC_CTRL0];
  memset(pmscctrl0, 0, LEN_PMSC_CTRL0);
  readBytes(PMSC, PMSC_CTRL0_SUB, pmscctrl0, LEN_PMSC_CTRL0);
  if(clock == AUTO_CLOCK) {
    _currentSPI = &_fastSPI;
    pmscctrl0[0] = AUTO_CLOCK;
    pmscctrl0[1] &= 0xFE;
  } else if(clock == XTI_CLOCK) {
    _currentSPI = &_slowSPI;
    pmscctrl0[0] &= 0xFC;
    pmscctrl0[0] |= XTI_CLOCK;
  } else if(clock == PLL_CLOCK) {
    _currentSPI = &_fastSPI;
    pmscctrl0[0] &= 0xFC;
    pmscctrl0[0] |= PLL_CLOCK;
  } else {
    // TODO deliver proper warning
  }
  writeBytes(PMSC, PMSC_CTRL0_SUB, pmscctrl0, 2);
}

void reset() {
  if (_rst == 0xff) {
    return;
  }

  // DW3000 RSTn is open-drain. Drive it low, then release it.
  // A weak pull-up lets the pin rise without pushing it high.
  pinMode(_rst, OUTPUT);
  digitalWrite(_rst, LOW);
  delay(2);
  pinMode(_rst, INPUT_PULLUP);
  delay(20);
}

void softReset() {
  byte pmscctrl0[LEN_PMSC_CTRL0];
  readBytes(PMSC, PMSC_CTRL0_SUB, pmscctrl0, LEN_PMSC_CTRL0);
  pmscctrl0[0] = 0x01;
  writeBytes(PMSC, PMSC_CTRL0_SUB, pmscctrl0, LEN_PMSC_CTRL0);
  pmscctrl0[3] = 0x00;
  writeBytes(PMSC, PMSC_CTRL0_SUB, pmscctrl0, LEN_PMSC_CTRL0);
  delay(10);
  pmscctrl0[0] = 0x00;
  pmscctrl0[3] = 0xF0;
  writeBytes(PMSC, PMSC_CTRL0_SUB, pmscctrl0, LEN_PMSC_CTRL0);
  // force into idle mode
  idle();
}

void setBit(byte data[], uint16_t n, uint16_t bit, boolean val) {
  uint16_t idx;
  uint8_t shift;
  
  idx = bit/8;
  if(idx >= n) {
    return; // TODO proper error handling: out of bounds
  }
  byte* targetByte = &data[idx];
  shift = bit%8;
  if(val) {
    bitSet(*targetByte, shift);
  } else {
    bitClear(*targetByte, shift);
  }
}

/*
 * Check the value of a bit in an array of bytes that are considered
 * consecutive and stored from MSB to LSB.
 * @param data
 *    The number as byte array.
 * @param n
 *    The number of bytes in the array.
 * @param bit
 *    The position of the bit to be checked.
 */
boolean getBit(byte data[], uint16_t n, uint16_t bit) {
  uint16_t idx;
  uint8_t  shift;
  
  idx = bit/8;
  if(idx >= n) {
    return false; // TODO proper error handling: out of bounds
  }
  byte targetByte = data[idx];
  shift = bit%8;
  
  return bitRead(targetByte, shift); // TODO wrong type returned byte instead of boolean
}

void writeValueToBytes(byte data[], int32_t val, uint16_t n) {
  uint16_t i;
  for(i = 0; i < n; i++) {
    data[i] = ((val >> (i*8)) & 0xFF); // TODO bad types - signed unsigned problem
  }
}

void readSystemConfigurationRegister() {
  readBytes(SYS_CFG, NO_SUB, _syscfg, LEN_SYS_CFG);
}

void writeSystemConfigurationRegister() {
  writeBytes(SYS_CFG, NO_SUB, _syscfg, LEN_SYS_CFG);
}

void readSystemEventStatusRegister() {
  readBytes(SYS_STATUS, NO_SUB, _sysstatus, LEN_SYS_STATUS);
}

void readNetworkIdAndDeviceAddress() {
  readBytes(PANADR, NO_SUB, _networkAndAddress, LEN_PANADR);
}

void writeNetworkIdAndDeviceAddress() {
  writeBytes(PANADR, NO_SUB, _networkAndAddress, LEN_PANADR);
}

void readSystemEventMaskRegister() {
  readBytes(SYS_MASK, NO_SUB, _sysmask, LEN_SYS_MASK);
}

void writeSystemEventMaskRegister() {
  writeBytes(SYS_MASK, NO_SUB, _sysmask, LEN_SYS_MASK);
}

void readChannelControlRegister() {
  readBytes(CHAN_CTRL, NO_SUB, _chanctrl, LEN_CHAN_CTRL);
}

void writeChannelControlRegister() {
  writeBytes(CHAN_CTRL, NO_SUB, _chanctrl, LEN_CHAN_CTRL);
}

void readTransmitFrameControlRegister() {
  readBytes(TX_FCTRL, NO_SUB, _txfctrl, LEN_TX_FCTRL);
}

void writeTransmitFrameControlRegister() {
  writeBytes(TX_FCTRL, NO_SUB, _txfctrl, LEN_TX_FCTRL);
}

void idle() {
  memset(_sysctrl, 0, LEN_SYS_CTRL);
  setBit(_sysctrl, LEN_SYS_CTRL, TRXOFF_BIT, true);
  _deviceMode = IDLE_MODE;
  writeBytes(SYS_CTRL, NO_SUB, _sysctrl, LEN_SYS_CTRL);
}

void setDoubleBuffering(boolean val) {
  setBit(_syscfg, LEN_SYS_CFG, DIS_DRXB_BIT, !val);
}

void setInterruptPolarity(boolean val) {
  setBit(_syscfg, LEN_SYS_CFG, HIRQ_POL_BIT, val);
}

void clearInterrupts() {
  memset(_sysmask, 0, LEN_SYS_MASK);
}

void manageLDE() {
  // transfer any ldo tune values
  byte ldoTune[LEN_OTP_RDAT];
  readBytesOTP(0x04, ldoTune); // TODO #define
  if(ldoTune[0] != 0) {
    // TODO tuning available, copy over to RAM: use OTP_LDO bit
  }
  // tell the chip to load the LDE microcode
  // TODO remove clock-related code (PMSC_CTRL) as handled separately
  byte pmscctrl0[LEN_PMSC_CTRL0];
  byte otpctrl[LEN_OTP_CTRL];
  memset(pmscctrl0, 0, LEN_PMSC_CTRL0);
  memset(otpctrl, 0, LEN_OTP_CTRL);
  readBytes(PMSC, PMSC_CTRL0_SUB, pmscctrl0, LEN_PMSC_CTRL0);
  readBytes(OTP_IF, OTP_CTRL_SUB, otpctrl, LEN_OTP_CTRL);
  pmscctrl0[0] = 0x01;
  pmscctrl0[1] = 0x03;
  otpctrl[0]   = 0x00;
  otpctrl[1]   = 0x80;
  writeBytes(PMSC, PMSC_CTRL0_SUB, pmscctrl0, 2);
  writeBytes(OTP_IF, OTP_CTRL_SUB, otpctrl, 2);
  delay(5);
  pmscctrl0[0] = 0x00;
  pmscctrl0[1] &= 0x02;
  writeBytes(PMSC, PMSC_CTRL0_SUB, pmscctrl0, 2);
}

void Sleep(uint32_t d) {
    delay(d);
}

#if defined(ARDUINO_ARCH_STM32)
static GPIO_TypeDef *_bb_sck_port;
static GPIO_TypeDef *_bb_mosi_port;
static GPIO_TypeDef *_bb_miso_port;
static uint32_t _bb_sck_bit;
static uint32_t _bb_mosi_bit;
static uint32_t _bb_miso_bit;

static void stm32_bb_wait(void)
{
  /* ~10 us at 72 MHz. delayMicroseconds(1) returns immediately on this core,
   * so MOSI was still changing on the same edge as SCK. */
  for (volatile uint16_t n = 0; n < 40; n++) {
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
    UART_puts("SPI pin fail\r\n");
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

static void dw_spi_header(const uint8_t *header, uint16_t headerLength)
{
  for (uint16_t i = 0; i < headerLength; i++) {
    dw_xfer(header[i]);
  }
}

static uint8_t dw_spi_byte(uint8_t data)
{
  return dw_xfer(data);
}

static void dw_spi_close(void)
{
  delayMicroseconds(5);
  digitalWrite(_ss, HIGH);
#if !defined(ARDUINO_ARCH_STM32)
  DW_SPI_PORT.endTransaction();
#endif
}

void spiSelect(uint8_t ss) {
  reselect(ss);
  // Hardware reset only. DW3000 register setup is done by dwt_initialise().
  reset();
}

void spiSetFastRate(void) {
#if !defined(ARDUINO_ARCH_STM32)
  _currentSPI = &_fastSPI;
#endif
}


int readfromspi(uint16_t headerLength, uint8_t *headerBuffer, uint16_t readLength, uint8_t *readBuffer)
{
  dw_spi_open();
  dw_spi_header(headerBuffer, headerLength);
  for(int i = 0; i < readLength; i++) {
    readBuffer[i] = dw_spi_byte(JUNK); // read values
  }
  dw_spi_close();




  
  /*open_spi(); // we first open the SPI line by setting it to low
  for(int i=0; i<headerLength; i++) // write our header bytes to selected chip
  {
    spi_tranceiver(headerBuffer[i]);
  }
  for(int i=0; i<readLength; i++)
  {
    readBuffer[i] = spi_tranceiver(0x00); // store read byte value in the buffer
  
  }
  sleepus(5); // not sure if this is needed?
  close_spi(); // close the SPI line by setting it to high*/
  return 0;
}

int writetospi(uint16_t headerLength, uint8_t *headerBuffer, uint16_t bodyLength, uint8_t *bodyBuffer)
{
  dw_spi_open();
  dw_spi_header(headerBuffer, headerLength);
  for(int i = 0; i < bodyLength; i++) {
    dw_spi_byte(bodyBuffer[i]); // write values
  }
  dw_spi_close();

  
  /*open_spi(); // we first open the SPI line by setting it to low
  for(int i=0; i<headerLength; i++) // write our header bytes to selected chip
  {
    spi_tranceiver(headerBuffer[i]);  
    
  }
  for(int i=0; i<bodyLength; i++) // write our body data bytes to the selected chip
  {
    spi_tranceiver(bodyBuffer[i]);
    
  }
  sleepus(5); // not sure if this is needed?
  close_spi(); // close the SPI line by setting it to high*/
  return 0;
}

void wakeup_device_with_io() {
    digitalWrite(_ss, LOW);
    delay(2);
    digitalWrite(_ss, HIGH);
    if (_debounceClockEnabled){
            enableDebounceClock();
    }
}


void port_set_dw_ic_spi_fastrate(uint8_t irq, uint8_t rst, uint8_t ss) {
    spiBegin(irq, rst);
    spiSelect(ss);
}

uint32_t port_GetEXT_IRQStatus(void) {
    return _irq_enabled ? 1UL : 0UL;
}

uint32_t port_CheckEXT_IRQ(void) {
    return digitalRead(_irq) ? 1UL : 0UL;
}

void port_DisableEXT_IRQ(void) {
    _irq_enabled = false;
}

void port_EnableEXT_IRQ(void) {
    _irq_enabled = true;
}

/* DW IC IRQ handler definition. */
static port_dwic_isr_t port_dwic_isr = NULL;

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn port_set_dwic_isr()
 *
 * @brief This function is used to install the handling function for DW IC IRQ.
 *
 * NOTE:
 *   - The user application shall ensure that a proper handler is set by calling this function before any DW IC IRQ occurs.
 *   - This function deactivates the DW IC IRQ line while the handler is installed.
 *
 * @param deca_isr function pointer to DW IC interrupt handler to install
 *
 * @return none
 */
void port_set_dwic_isr(port_dwic_isr_t dwic_isr)
{
    /* Check DW IC IRQ activation status. */
    //ITStatus en = port_GetEXT_IRQStatus();

    /* If needed, deactivate DW IC IRQ during the installation of the new handler. */
    //port_DisableEXT_IRQ();
    portDISABLE_INTERRUPTS();
    port_dwic_isr = dwic_isr;
    portENABLE_INTERRUPTS();
/*
    if (!en)
    {
        port_EnableEXT_IRQ();
    }*/
}


#if 0
void open_spi(void)
{
  //PORTB &= ~_BV(PORTB2); // set SS pin to LOW to enable SPI
}

void close_spi(void)
{
  //PORTB |= _BV(PORTB2); // set SS pin to HIGH to disable SPI
}

int spi_tranceiver (uint8_t *data) // send single byte
{
  /*SPDR0 = data; // Load data into the buffer
  sleepus(1);
  while(!(SPSR0 & _BV(SPIF) )); // Wait until transmission complete
  
  // Return received data
  return(SPDR0);*/
  return (0);
}



void port_set_dw_ic_spi_slowrate(void)
{
  //SPSR0 &= ~_BV(SPI2X); // turn off fast speed
}

void port_set_dw_ic_spi_fastrate(void)
{
  //SPSR0 |= _BV(SPI2X); // set fast speed by changing oscillator speed to FOSC / 2
}

void reset_DWIC(void) // currently not used as we are using softreset()
{
  /*DDR_PORTD |= _BV(DD_RESET_PIN); // set reset PIN as output
  PORTD &= ~_BV(PORTD7); // set reset pin to low for brief amount of time

  sleepus(1);

  DDR_PORTD &= ~_BV(DD_RESET_PIN); // set reset pin to input again

  sleepms(2); // allow for chip to turn back on*/

}

#endif
