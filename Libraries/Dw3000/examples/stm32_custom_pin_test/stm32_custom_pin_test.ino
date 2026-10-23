#include "dw3000.h"

#define APP_NAME "STM32 DWM3000 PIN TEST v1.0"
/*
 * Pin-level check for the DWM3000 link when the library reads FFFFFFFF.
 * Uses the DW3000_PIN_* wiring from dw3000_board_config.h and its own slow
 * bit-banged SPI, so nothing in the driver is involved.
 * Console USART1 PA9/PA10, 115200.
 *
 * 1. Line levels: MISO with pull-down/pull-up, CS high and low, RSTn, IRQ.
 * 2. Hardware reset, then a DEV_ID read with MOSI/MISO as wired and swapped.
 *    Expected ID DECA0302 or DECA0312.
 * 3. Loop: drives SCK, MOSI and CS high one at a time for 3 s each, so the
 *    level can be measured on the module pins with a multimeter.
 */

static const uint8_t PIN_SCK = DW3000_PIN_SCK;
static const uint8_t PIN_MOSI = DW3000_PIN_MOSI;
static const uint8_t PIN_MISO = DW3000_PIN_MISO;
static const uint8_t PIN_CS = DW3000_PIN_CS;
static const uint8_t PIN_RST = DW3000_PIN_RST;
static const uint8_t PIN_IRQ = DW3000_PIN_IRQ;

static void say(const char *msg)
{
  test_run_info((unsigned char *)msg);
}

/* About 25 us per half clock at 72 MHz: far below any DW3000 limit. */
static void slow_wait(void)
{
  for (volatile uint16_t n = 0; n < 300; n++) {
    __NOP();
  }
}

static uint8_t slow_transfer(uint8_t mosi, uint8_t miso, uint8_t tx)
{
  uint8_t rx = 0;
  for (uint8_t bit = 0; bit < 8; bit++) {
    digitalWrite(mosi, (tx & 0x80) ? HIGH : LOW);
    tx <<= 1;
    slow_wait();
    digitalWrite(PIN_SCK, HIGH);
    slow_wait();
    rx = (rx << 1) | (digitalRead(miso) ? 1 : 0);
    digitalWrite(PIN_SCK, LOW);
  }
  return rx;
}

/* Short read of register 0x00 (DEV_ID). The ID comes back LSB first. */
static void read_dev_id(uint8_t mosi, uint8_t miso, const char *label)
{
  pinMode(mosi, OUTPUT);
  pinMode(miso, INPUT);
  digitalWrite(PIN_SCK, LOW);
  digitalWrite(mosi, LOW);

  uint8_t b[4];
  digitalWrite(PIN_CS, LOW);
  slow_wait();
  slow_transfer(mosi, miso, 0x00);
  for (uint8_t i = 0; i < 4; i++) {
    b[i] = slow_transfer(mosi, miso, 0x00);
  }
  slow_wait();
  digitalWrite(PIN_CS, HIGH);

  uint32_t id = ((uint32_t)b[3] << 24) | ((uint32_t)b[2] << 16) | ((uint32_t)b[1] << 8) | b[0];
  char line[64];
  snprintf(line, sizeof(line), "%s ID %08lX %s", label, (unsigned long)id,
           (id == DWT_C0_DEV_ID || id == DWT_C0_PDOA_DEV_ID) ? "OK" : "BAD");
  say(line);
}

static void report_levels(void)
{
  char line[64];
  pinMode(PIN_MISO, INPUT_PULLDOWN);
  digitalWrite(PIN_CS, HIGH);
  delay(2);
  int cs_hi_pd = digitalRead(PIN_MISO);
  digitalWrite(PIN_CS, LOW);
  delay(2);
  int cs_lo_pd = digitalRead(PIN_MISO);
  pinMode(PIN_MISO, INPUT_PULLUP);
  delay(2);
  int cs_lo_pu = digitalRead(PIN_MISO);
  digitalWrite(PIN_CS, HIGH);
  pinMode(PIN_MISO, INPUT);

  snprintf(line, sizeof(line), "MISO CS=1 pd:%d  CS=0 pd:%d pu:%d", cs_hi_pd, cs_lo_pd, cs_lo_pu);
  say(line);
  if (cs_hi_pd) {
    say("  MISO high with CS high: shorted to 3V3 or a pin is wrong");
  } else if (cs_lo_pd == 0 && cs_lo_pu == 1) {
    say("  MISO follows the pull: the module never drives it");
  }

  snprintf(line, sizeof(line), "RSTn %d  IRQ %d", digitalRead(PIN_RST), digitalRead(PIN_IRQ));
  say(line);
  if (digitalRead(PIN_RST) == 0) {
    say("  RSTn low: module held in reset or unpowered");
  }
}

void setup()
{
  UART_init();
  say(APP_NAME);
#if defined(STM32F103xE)
  say(dw3000_clock_from_hse ? "CLOCK HSE 72 MHz" : "CLOCK HSI 64 MHz (crystal failed)");
#endif

  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
  pinMode(PIN_SCK, OUTPUT);
  digitalWrite(PIN_SCK, LOW);
  pinMode(PIN_MOSI, OUTPUT);
  digitalWrite(PIN_MOSI, LOW);
  pinMode(PIN_IRQ, INPUT);
  pinMode(PIN_RST, INPUT_PULLUP);
  delay(10);

  say("-- levels before reset");
  report_levels();

  /* RSTn is open-drain: pull low, then release to the pull-up. */
  pinMode(PIN_RST, OUTPUT);
  digitalWrite(PIN_RST, LOW);
  delay(2);
  pinMode(PIN_RST, INPUT_PULLUP);
  delay(20);

  say("-- levels after reset");
  report_levels();

  say("-- DEV_ID reads");
  read_dev_id(PIN_MOSI, PIN_MISO, "wired  ");
  read_dev_id(PIN_MISO, PIN_MOSI, "swapped");
  pinMode(PIN_MOSI, OUTPUT);
  pinMode(PIN_MISO, INPUT);
  digitalWrite(PIN_MOSI, LOW);

  say("-- toggle test: measure on the module pins");
}

void loop()
{
  static const uint8_t pins[] = {PIN_SCK, PIN_MOSI, PIN_CS};
  static const char *names[] = {"SCK high, MOSI low, CS high", "MOSI high, SCK low, CS high", "CS low, SCK low, MOSI low"};
  for (uint8_t i = 0; i < 3; i++) {
    digitalWrite(PIN_SCK, LOW);
    digitalWrite(PIN_MOSI, LOW);
    digitalWrite(PIN_CS, HIGH);
    digitalWrite(pins[i], (pins[i] == PIN_CS) ? LOW : HIGH);
    say(names[i]);
    delay(3000);
  }
  digitalWrite(PIN_CS, HIGH);
}
