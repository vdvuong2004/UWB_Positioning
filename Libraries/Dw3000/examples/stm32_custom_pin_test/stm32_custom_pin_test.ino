#include "dw3000.h"

#define APP_NAME "STM32 DWM3000 BOARD TEST v2.2"
/*
 * Bring-up test for the custom STM32F103RCT6 + DWM3000 board, logged over
 * the board's own USB port (PA11/PA12). USART1 PA9/PA10 gets the same text.
 *
 * Arduino IDE: Board "Generic STM32F1 series", part "Generic F103RCTx",
 * Tools > USB support: "CDC (generic 'Serial' supersede U(S)ART)".
 * Open the board's USB COM port at any baud; the test waits up to 5 s for it
 * and runs again whenever the port is (re)opened.
 *
 * 1. Line levels: MISO with pull-down/pull-up, CS high and low, RSTn, IRQ.
 * 2. Bit-banged DEV_ID read with MOSI/MISO as wired and swapped, no driver.
 * 3. Driver: IDLE_RC, DEV_ID, 1000 repeated DEV_ID reads, dwt_initialise,
 *    part/lot ID, temperature and supply, dwt_configure (PLL lock), one TX.
 * 4. Loop: DEV_ID, temperature and supply every 2 s.
 *
 * Commands over USB: r = run all tests again, t = pin toggle test for a
 * multimeter (SCK, MOSI, CS for 3 s each), s = stop/start the 2 s report.
 */

#if !defined(ARDUINO_ARCH_STM32) || !defined(STM32F103xE)
#error "Select Generic STM32F1 series, part Generic F103RCTx."
#endif
#if !defined(USBCON) || !defined(USBD_USE_CDC)
#error "Tools > USB support: CDC (generic 'Serial' supersede U(S)ART)."
#endif

static const uint8_t PIN_SCK = DW3000_PIN_SCK;
static const uint8_t PIN_MOSI = DW3000_PIN_MOSI;
static const uint8_t PIN_MISO = DW3000_PIN_MISO;
static const uint8_t PIN_CS = DW3000_PIN_CS;
static const uint8_t PIN_RST = DW3000_PIN_RST;
static const uint8_t PIN_IRQ = DW3000_PIN_IRQ;

/* Same radio settings as the ranging sketches. */
static dwt_config_t config = {
    5,               /* Channel number. */
    DWT_PLEN_128,    /* Preamble length. */
    DWT_PAC8,        /* Preamble acquisition chunk size. */
    9,               /* TX preamble code. */
    9,               /* RX preamble code. */
    0,               /* IEEE 8-symbol SFD. */
    DWT_BR_6M8,      /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    DWT_PHRRATE_STD, /* PHY header rate. */
    (129 + 8 - 8),   /* SFD timeout. */
    DWT_STS_MODE_OFF,
    DWT_STS_LEN_64,
    DWT_PDOA_M0
};

static dwt_txconfig_t txconfig = {
    0x34,       /* PG delay, channel 5. */
    0x48484848, /* TX power. Low on purpose: bench test only. */
    0x0
};

static uint8_t tx_msg[] = {0xC5, 0, 'B', 'O', 'A', 'R', 'D', 'T', 'S', 'T'};

static bool driver_ready = false;
static bool report_on = true;

static void say(const char *msg)
{
  test_run_info((unsigned char *)msg);
}

static void sayf(const char *fmt, ...)
{
  char line[96];
  va_list args;
  va_start(args, fmt);
  vsnprintf(line, sizeof(line), fmt, args);
  va_end(args);
  say(line);
}

static bool id_ok(uint32_t id)
{
  return id == DWT_C0_DEV_ID || id == DWT_C0_PDOA_DEV_ID;
}

/* Logs printed before the host opens the port are lost, so wait for DTR. */
static void wait_usb_host(void)
{
  uint32_t start = millis();
  while (!SerialUSB && (millis() - start < 5000)) {
    delay(10);
  }
  delay(100);
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
static bool read_dev_id(uint8_t mosi, uint8_t miso, const char *label)
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
  sayf("%s ID %08lX %s", label, (unsigned long)id, id_ok(id) ? "OK" : "BAD");
  return id_ok(id);
}

static void report_levels(void)
{
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

  sayf("MISO CS=1 pd:%d  CS=0 pd:%d pu:%d", cs_hi_pd, cs_lo_pd, cs_lo_pu);
  if (cs_hi_pd) {
    say("  MISO high with CS high: shorted to 3V3 or a pin is wrong");
  } else if (cs_lo_pd == 0 && cs_lo_pu == 1) {
    say("  MISO follows the pull: the module never drives it");
  }

  sayf("RSTn %d  IRQ %d", digitalRead(PIN_RST), digitalRead(PIN_IRQ));
  if (digitalRead(PIN_RST) == 0) {
    say("  RSTn low: module held in reset or unpowered");
  }
}

/* Raw pin checks, no driver involved. */
static bool pin_tests(void)
{
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

  say("-- bit-banged DEV_ID");
  bool wired = read_dev_id(PIN_MOSI, PIN_MISO, "wired  ");
  bool swapped = read_dev_id(PIN_MISO, PIN_MOSI, "swapped");
  pinMode(PIN_MOSI, OUTPUT);
  pinMode(PIN_MISO, INPUT);
  digitalWrite(PIN_MOSI, LOW);
  if (!wired && swapped) {
    say("  MOSI and MISO are swapped on the board");
  }
  return wired;
}

/* Bit-banged SPI glitches show up as a wrong ID now and then. */
static bool spi_stress(void)
{
  const uint16_t reads = 1000;
  uint16_t bad = 0;
  uint32_t last_bad = 0;
  uint32_t start = millis();
  for (uint16_t i = 0; i < reads; i++) {
    uint32_t id = dwt_readdevid();
    if (!id_ok(id)) {
      bad++;
      last_bad = id;
    }
  }
  sayf("SPI %u DEV_ID reads in %lu ms, %u bad", reads, (unsigned long)(millis() - start), bad);
  if (bad) {
    sayf("  last bad read %08lX", (unsigned long)last_bad);
  }
  return bad == 0;
}

/* Last readings, also readable over SWD without USB:
 * STM32_Programmer_CLI -c port=SWD mode=HOTPLUG -r32 <address of board_diag> 20 */
extern "C" volatile struct {
  uint32_t magic;     /* 0xB0A2D1A6 once filled */
  uint32_t dev_id;
  uint32_t dw_raw;    /* temp << 8 | vbat SAR codes */
  uint32_t dw_ref;    /* OTP temp ref << 8 | vbat ref */
  uint32_t mcu_mv;    /* STM32 VDDA from VREFINT */
} board_diag;
volatile decltype(board_diag) board_diag = {};

/* F1 has no VREFINT calibration: 1.20 V typical, so about +-3 %. */
static uint32_t mcu_vdda_mv(void)
{
  analogReadResolution(12);
  uint32_t sum = 0;
  for (uint8_t i = 0; i < 8; i++) {
    sum += analogRead(AVREF);
  }
  return sum ? (1200UL * 4095UL * 8UL) / sum : 0;
}

static void report_temp_vbat(void)
{
  uint16_t raw = dwt_readtempvbat();
  float temp = dwt_convertrawtemperature((uint8_t)(raw >> 8));
  float vbat = dwt_convertrawvoltage((uint8_t)(raw & 0xFF));
  uint32_t mcu_mv = mcu_vdda_mv();
  /* newlib-nano printf has no %f. */
  sayf("TEMP %d.%d C  DW VDD %d.%02d V (sar %02X ref %02X)  MCU VDD %lu.%02lu V",
       (int)temp, abs((int)(temp * 10)) % 10, (int)vbat, abs((int)(vbat * 100)) % 100,
       raw & 0xFF, dwt_geticrefvolt(), (unsigned long)(mcu_mv / 1000), (unsigned long)(mcu_mv % 1000 / 10));

  board_diag.dev_id = dwt_readdevid();
  board_diag.dw_raw = raw;
  board_diag.dw_ref = ((uint32_t)dwt_geticreftemp() << 8) | dwt_geticrefvolt();
  board_diag.mcu_mv = mcu_mv;
  board_diag.magic = 0xB0A2D1A6;
}

static bool tx_test(void)
{
  dwt_writetxdata(sizeof(tx_msg), tx_msg, 0);
  dwt_writetxfctrl(sizeof(tx_msg) + FCS_LEN, 0, 0);
  dwt_starttx(DWT_START_TX_IMMEDIATE);

  uint32_t start = millis();
  while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK)) {
    if (millis() - start > 100) {
      dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_TX);
      return false;
    }
  }
  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
  tx_msg[1]++;
  return true;
}

static bool check(bool ok, const char *name)
{
  sayf("[%s] %s", ok ? "PASS" : "FAIL", name);
  return ok;
}

/* Driver path, same init order as the other sketches. spiSelect() pulses RSTn. */
static bool driver_tests(void)
{
  driver_ready = false;
  spiBegin(PIN_IRQ, PIN_RST);
  spiSelect(PIN_CS);

  bool idle = false;
  for (int attempt = 0; attempt < 20 && !idle; attempt++) {
    idle = dwt_checkidlerc();
    delay(2);
  }
  if (!check(idle, "IDLE_RC after reset")) {
    return false;
  }

  uint32_t id = dwt_readdevid();
  sayf("DEV_ID %08lX", (unsigned long)id);
  if (!check(id_ok(id), "driver DEV_ID")) {
    return false;
  }
  check(spi_stress(), "SPI repeated reads");

  if (!check(dwt_initialise(DWT_DW_INIT) != DWT_ERROR, "dwt_initialise")) {
    return false;
  }
  sayf("PART ID %08lX  LOT ID %08lX", (unsigned long)dwt_getpartid(), (unsigned long)dwt_getlotid());
  report_temp_vbat();

  if (!check(dwt_configure(&config) == DWT_SUCCESS, "dwt_configure (PLL lock, RX cal)")) {
    return false;
  }
  dwt_configuretxrf(&txconfig);
  spiSetFastRate();
  driver_ready = true;

  return check(tx_test(), "TX one frame");
}

static void run_all(void)
{
  say("");
  say(APP_NAME);
  say(dw3000_clock_from_hse ? "CLOCK HSE 72 MHz" : "CLOCK HSI 48 MHz (crystal failed)");
  sayf("PINS SCK %u MOSI %u MISO %u CS %u RST %u IRQ %u (Arduino pin numbers)",
       PIN_SCK, PIN_MOSI, PIN_MISO, PIN_CS, PIN_RST, PIN_IRQ);

  say("== 1/2 pin test");
  bool pins = pin_tests();
  check(pins, "bit-banged DEV_ID");

  say("== 3 driver test");
  bool drv = driver_tests();

  say(pins && drv ? "== RESULT: BOARD OK" : "== RESULT: BOARD FAILED");
  say("commands: r = rerun, t = toggle test, s = report on/off");
}

/* Drives SCK, MOSI and CS one at a time for 3 s each, to measure on the module pins. */
static void toggle_test(void)
{
  say("-- toggle test: measure on the module pins");
  driver_ready = false;
  pinMode(PIN_SCK, OUTPUT);
  pinMode(PIN_MOSI, OUTPUT);
  pinMode(PIN_CS, OUTPUT);
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
  digitalWrite(PIN_SCK, LOW);
  digitalWrite(PIN_MOSI, LOW);
  digitalWrite(PIN_CS, HIGH);
  say("-- toggle done, r to rerun the tests");
}

void setup()
{
  UART_init();
  wait_usb_host();
  run_all();
}

void loop()
{
  static uint32_t last = 0;
  static bool host_open = SerialUSB.dtr();

  /* Upload resets the board and drops the COM port, so a terminal is often
   * reopened after the first run. Run again each time the port is opened. */
  bool dtr = SerialUSB.dtr();
  if (dtr && !host_open) {
    delay(100);
    run_all();
  }
  host_open = dtr;

  while (SerialUSB.available()) {
    char c = SerialUSB.read();
    if (c == 'r' || c == 'R') {
      run_all();
    } else if (c == 't' || c == 'T') {
      toggle_test();
    } else if (c == 's' || c == 'S') {
      report_on = !report_on;
      say(report_on ? "report on" : "report off");
    }
  }

  if (report_on && driver_ready && millis() - last >= 2000) {
    last = millis();
    uint32_t id = dwt_readdevid();
    sayf("DEV_ID %08lX %s", (unsigned long)id, id_ok(id) ? "OK" : "BAD");
    report_temp_vbat();
  }
}
