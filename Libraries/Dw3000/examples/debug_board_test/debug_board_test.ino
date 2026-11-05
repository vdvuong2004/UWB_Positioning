#include "dw3000.h"
#include <stdarg.h>

#define APP_NAME "DEBUG BOARD TEST v3.0"
/*
 * Connection test: custom STM32F103RCT6 board + DWM3000 on J7/J8.
 * Prints OK / FAIL / ?? for every net (3V3, GND, CS, MISO, SCK, MOSI, RSTn, IRQ),
 * then runs the driver and one TX.
 * Board: Generic F103RCTx, USB support: CDC. Log on the board's USB port.
 * Commands: r = rerun, t = hold each line 5 s for a multimeter, s = report on/off.
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

static dwt_config_t config = {
    5, DWT_PLEN_128, DWT_PAC8, 9, 9, 0,
    DWT_BR_6M8, DWT_PHRMODE_STD, DWT_PHRRATE_STD,
    (129 + 8 - 8), DWT_STS_MODE_OFF, DWT_STS_LEN_64, DWT_PDOA_M0
};
static dwt_txconfig_t txconfig = {0x34, 0x48484848, 0x0};
static uint8_t tx_msg[] = {0xC5, 0, 'B', 'O', 'A', 'R', 'D', 'T', 'S', 'T'};

enum verdict_t { V_SKIP, V_OK, V_FAIL, V_UNSURE };
struct net_t {
  const char *name;
  verdict_t v;
  char hint[72];
};
enum { N_3V3, N_GND, N_CS, N_MISO, N_SCK, N_MOSI, N_RST, N_IRQ, N_COUNT };
static net_t nets[N_COUNT] = {
  {"3V3"}, {"GND"}, {"CS   PA15"}, {"MISO PB4"}, {"SCK  PB3"}, {"MOSI PB5"}, {"RSTn PD2"}, {"IRQ  PC12"},
};

static bool driver_ready = false;
static bool report_on = true;
static uint32_t enable_mask = 0;

static void say(const char *msg)
{
  test_run_info((unsigned char *)msg);
}

static void sayf(const char *fmt, ...)
{
  char line[112];
  va_list args;
  va_start(args, fmt);
  vsnprintf(line, sizeof(line), fmt, args);
  va_end(args);
  say(line);
}

/* The first failure is usually the cause, so it is not overwritten. */
static void set_net(uint8_t n, verdict_t v, const char *hint)
{
  if (nets[n].v == V_FAIL) {
    return;
  }
  nets[n].v = v;
  snprintf(nets[n].hint, sizeof(nets[n].hint), "%s", hint);
}

static bool id_ok(uint32_t id)
{
  return id == DWT_C0_DEV_ID || id == DWT_C0_PDOA_DEV_ID;
}

static void wait_usb_host(void)
{
  uint32_t start = millis();
  while (!SerialUSB && (millis() - start < 5000)) {
    delay(10);
  }
  delay(100);
}

/* VREFINT 1.20 V typical (F1 has no calibration), about +-3 %. */
static uint32_t mcu_vdd_mv(void)
{
  analogReadResolution(12);
  uint32_t sum = 0;
  for (uint8_t i = 0; i < 8; i++) {
    sum += analogRead(AVREF);
  }
  return sum ? (1200UL * 4095UL * 8UL) / sum : 0;
}

static void idle_lines(void)
{
  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
  pinMode(PIN_SCK, OUTPUT);
  digitalWrite(PIN_SCK, LOW);
  pinMode(PIN_MOSI, OUTPUT);
  digitalWrite(PIN_MOSI, LOW);
  pinMode(PIN_MISO, INPUT);
  pinMode(PIN_IRQ, INPUT);
  pinMode(PIN_RST, INPUT_PULLUP);
}

static const char *pin_name(uint8_t p)
{
  if (p == PIN_SCK) return "SCK";
  if (p == PIN_MOSI) return "MOSI";
  if (p == PIN_MISO) return "MISO";
  if (p == PIN_CS) return "CS";
  if (p == PIN_RST) return "RSTn";
  return "IRQ";
}

static uint8_t net_of(uint8_t p)
{
  if (p == PIN_SCK) return N_SCK;
  if (p == PIN_MOSI) return N_MOSI;
  if (p == PIN_MISO) return N_MISO;
  if (p == PIN_CS) return N_CS;
  if (p == PIN_RST) return N_RST;
  return N_IRQ;
}

/* b follows a both ways, against a pull the other way: shorted. */
static bool follows(uint8_t a, uint8_t b)
{
  pinMode(a, OUTPUT);
  digitalWrite(a, HIGH);
  pinMode(b, INPUT_PULLDOWN);
  delayMicroseconds(200);
  bool hi = digitalRead(b);
  digitalWrite(a, LOW);
  pinMode(b, INPUT_PULLUP);
  delayMicroseconds(200);
  bool lo = !digitalRead(b);
  return hi && lo;
}

static void short_test(void)
{
  say("-- shorts");
  /* CS low makes the module drive MISO, so that pair is skipped. */
  static const uint8_t drivers[] = {PIN_CS, PIN_SCK, PIN_MOSI};
  static const uint8_t receivers[] = {PIN_CS, PIN_SCK, PIN_MOSI, PIN_MISO, PIN_RST};
  bool any = false;
  for (uint8_t a : drivers) {
    for (uint8_t b : receivers) {
      if (a == b || (a == PIN_CS && b == PIN_MISO) || !follows(a, b)) {
        continue;
      }
      char hint[72];
      snprintf(hint, sizeof(hint), "shorted to %s", pin_name(b));
      set_net(net_of(a), V_FAIL, hint);
      snprintf(hint, sizeof(hint), "shorted to %s", pin_name(a));
      set_net(net_of(b), V_FAIL, hint);
      sayf("  %s - %s shorted", pin_name(a), pin_name(b));
      any = true;
    }
    idle_lines();
  }
  if (!any) {
    say("  none");
  }
}

/* 0 / 1 = driven, 2 = floating. */
static int miso_state(void)
{
  pinMode(PIN_MISO, INPUT_PULLDOWN);
  delayMicroseconds(300);
  int pd = digitalRead(PIN_MISO);
  pinMode(PIN_MISO, INPUT_PULLUP);
  delayMicroseconds(300);
  int pu = digitalRead(PIN_MISO);
  pinMode(PIN_MISO, INPUT);
  return (pd == 0 && pu == 1) ? 2 : pd;
}

static const char *state_name(int s)
{
  return s == 2 ? "floats" : (s ? "driven high" : "driven low");
}

static bool cs_miso_test(void)
{
  say("-- CS / MISO");
  idle_lines();
  int cs_hi = miso_state();
  digitalWrite(PIN_CS, LOW);
  delayMicroseconds(50);
  int cs_lo = miso_state();
  digitalWrite(PIN_CS, HIGH);
  sayf("  MISO with CS high: %s, CS low: %s", state_name(cs_hi), state_name(cs_lo));
  sayf("  RSTn %d, IRQ %d", digitalRead(PIN_RST), digitalRead(PIN_IRQ));

  if (cs_hi != 2) {
    set_net(N_MISO, V_FAIL, cs_hi ? "high while not selected: short to 3V3 / CS stuck low"
                                  : "low while not selected: short to GND / CS stuck low");
    set_net(N_CS, V_UNSURE, "module may be selected all the time");
    return false;
  }
  if (cs_lo == 2) {
    if (digitalRead(PIN_RST)) {
      set_net(N_CS, V_UNSURE, "module never selected: CS open?");
      set_net(N_MISO, V_UNSURE, "never driven: MISO open?");
    } else {
      set_net(N_3V3, V_FAIL, "RSTn low and MISO dead: module unpowered");
    }
    return false;
  }
  set_net(N_CS, V_OK, "selects the module");
  set_net(N_MISO, V_OK, "driven only while selected");
  return true;
}

/* Bit-banged reads glitch now and then: take a value seen twice. */
static uint32_t read_reg(uint16_t reg)
{
  uint32_t a = dwt_read32bitreg(reg);
  uint32_t b = dwt_read32bitreg(reg);
  if (a == b) {
    return a;
  }
  return (dwt_read32bitreg(reg) == b) ? b : a;
}

static bool dev_id_test(void)
{
  say("-- SCK (DEV_ID read)");
  uint32_t id = 0;
  for (uint8_t i = 0; i < 10 && !id_ok(id); i++) {
    id = dwt_readdevid();
    delay(2);
  }
  sayf("  DEV_ID %08lX (want DECA0302)", (unsigned long)id);
  if (id_ok(id)) {
    set_net(N_SCK, V_OK, "DEV_ID read");
    set_net(N_3V3, V_OK, "module answers");
    return true;
  }
  if (id == 0) {
    set_net(N_SCK, V_FAIL, "MISO stays low: no clock at the module");
  } else if (id == 0xFFFFFFFFUL) {
    set_net(N_SCK, V_UNSURE, "MISO stays high: no clock, or MISO pulled up");
  } else {
    set_net(N_SCK, V_UNSURE, "garbled ID: SCK/MISO contact");
  }
  return false;
}

/* SYS_ENABLE: plain read/write mask, no side effects. */
static bool mosi_test(void)
{
  say("-- MOSI (write / read back)");
  dwt_write32bitreg(SYS_ENABLE_LO_ID, 0);
  uint32_t r0 = read_reg(SYS_ENABLE_LO_ID);
  dwt_write32bitreg(SYS_ENABLE_LO_ID, 0xFFFFFFFFUL);
  uint32_t r1 = read_reg(SYS_ENABLE_LO_ID);
  enable_mask = r1 & ~r0;
  sayf("  write 0 -> %08lX, write FFFFFFFF -> %08lX", (unsigned long)r0, (unsigned long)r1);
  if (r0 == r1) {
    /* MOSI stuck low turns every header into a DEV_ID read. */
    set_net(N_MOSI, V_FAIL, id_ok(r0) ? "MOSI stuck low or open" : "writes do not land: MOSI open");
    dwt_write32bitreg(SYS_ENABLE_LO_ID, 0);
    return false;
  }
  static const uint32_t patterns[] = {0x55555555UL, 0xAAAAAAAAUL, 0x0F0F0F0FUL, 0xF0F0F0F0UL};
  uint8_t bad = 0;
  for (uint32_t p : patterns) {
    dwt_write32bitreg(SYS_ENABLE_LO_ID, p);
    uint32_t want = (p & enable_mask) | (r0 & ~enable_mask);
    uint32_t got = read_reg(SYS_ENABLE_LO_ID);
    if (got != want) {
      bad++;
      sayf("  wrote %08lX read %08lX want %08lX", (unsigned long)p, (unsigned long)got, (unsigned long)want);
    }
  }
  dwt_write32bitreg(SYS_ENABLE_LO_ID, 0);
  if (bad) {
    set_net(N_MOSI, V_FAIL, "some bits wrong: MOSI contact");
    return false;
  }
  set_net(N_MOSI, V_OK, "all patterns read back");
  return true;
}

static void rst_test(void)
{
  say("-- RSTn (reset clears the chip)");
  uint32_t pattern = (enable_mask & 0x55555555UL) ? (enable_mask & 0x55555555UL) : enable_mask;
  dwt_write32bitreg(SYS_ENABLE_LO_ID, pattern);
  uint32_t before = read_reg(SYS_ENABLE_LO_ID);
  reset();
  uint32_t after = read_reg(SYS_ENABLE_LO_ID);
  uint32_t id = dwt_readdevid();
  sayf("  before %08lX, after %08lX, DEV_ID %08lX", (unsigned long)before, (unsigned long)after, (unsigned long)id);
  if (!id_ok(id)) {
    set_net(N_RST, V_UNSURE, "module did not come back after reset");
  } else if (after == before) {
    set_net(N_RST, V_FAIL, "reset had no effect: RSTn open");
  } else {
    set_net(N_RST, V_OK, "reset clears the chip");
  }
  dwt_write32bitreg(SYS_ENABLE_LO_ID, 0);
}

/* After reset SPIRDY/RCINIT are set, so enabling them must raise IRQ. */
static void irq_test(void)
{
  say("-- IRQ (mask on / off)");
  uint32_t status = read_reg(SYS_STATUS_ID);
  uint32_t src = status & (SYS_STATUS_SPIRDY_BIT_MASK | SYS_STATUS_RCINIT_BIT_MASK) & enable_mask;
  dwt_write32bitreg(SYS_ENABLE_LO_ID, 0);
  delayMicroseconds(100);
  int off = digitalRead(PIN_IRQ);
  int on = -1;
  if (src) {
    dwt_write32bitreg(SYS_ENABLE_LO_ID, src);
    delayMicroseconds(100);
    on = digitalRead(PIN_IRQ);
    dwt_write32bitreg(SYS_ENABLE_LO_ID, 0);
  }
  sayf("  SYS_STATUS %08lX, IRQ mask off %d, on %d", (unsigned long)status, off, on);
  if (on < 0) {
    set_net(N_IRQ, V_UNSURE, "no status bit to trigger: press r");
  } else if (off == 0 && on == 1) {
    set_net(N_IRQ, V_OK, "follows the mask");
  } else if (off == on) {
    set_net(N_IRQ, V_FAIL, on ? "always high: short to 3V3" : "always low: IRQ open or short to GND");
  } else {
    set_net(N_IRQ, V_FAIL, "inverted");
  }
}

/* With GND (3V3) open the module runs on the IO pins' protection diodes and
 * resets once no line is low (high). */
static bool supply_hold(uint8_t level)
{
  uint32_t pattern = (enable_mask & 0xA5A5A5A5UL) ? (enable_mask & 0xA5A5A5A5UL) : enable_mask;
  dwt_write32bitreg(SYS_ENABLE_LO_ID, pattern);
  uint32_t before = read_reg(SYS_ENABLE_LO_ID);

  digitalWrite(PIN_CS, level);
  digitalWrite(PIN_SCK, level);
  digitalWrite(PIN_MOSI, level);
  pinMode(PIN_RST, level ? INPUT_PULLUP : INPUT);
  delay(500);
  idle_lines();
  delay(5);

  uint32_t after = read_reg(SYS_ENABLE_LO_ID);
  bool alive = id_ok(dwt_readdevid());
  sayf("  all lines %s 0.5 s: %08lX -> %08lX, DEV_ID %s", level ? "high" : "low",
       (unsigned long)before, (unsigned long)after, alive ? "OK" : "BAD");
  dwt_write32bitreg(SYS_ENABLE_LO_ID, 0);
  return alive && after == before;
}

static void supply_test(void)
{
  say("-- GND / 3V3");
  if (supply_hold(HIGH)) {
    set_net(N_GND, V_OK, "keeps power with all lines high");
  } else {
    set_net(N_GND, V_FAIL, "lost power: GND pin open");
  }
  if (supply_hold(LOW)) {
    set_net(N_3V3, V_OK, "keeps power with all lines low");
  } else {
    set_net(N_3V3, V_FAIL, "lost power: 3V3 pin open");
  }
}

/* DW VDD readings are wrong on DWM3000 (OTP reference), so only temperature. */
static void report_temp(void)
{
  uint16_t raw = dwt_readtempvbat();
  float temp = dwt_convertrawtemperature((uint8_t)(raw >> 8));
  sayf("TEMP %d.%d C", (int)temp, abs((int)(temp * 10)) % 10);
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

static bool driver_tests(void)
{
  driver_ready = false;
  reset();
  bool idle = false;
  for (int i = 0; i < 20 && !idle; i++) {
    idle = dwt_checkidlerc();
    delay(2);
  }
  if (!check(idle, "IDLE_RC after reset")) {
    return false;
  }
  uint16_t bad = 0;
  for (uint16_t i = 0; i < 1000; i++) {
    bad += !id_ok(dwt_readdevid());
  }
  sayf("1000 DEV_ID reads, %u bad", bad);
  check(bad == 0, "SPI repeated reads");
  if (!check(dwt_initialise(DWT_DW_INIT) != DWT_ERROR, "dwt_initialise")) {
    return false;
  }
  report_temp();
  if (!check(dwt_configure(&config) == DWT_SUCCESS, "dwt_configure (PLL lock, RX calibration)")) {
    say("  usually a weak module supply or header contact");
    return false;
  }
  dwt_configuretxrf(&txconfig);
  spiSetFastRate();
  driver_ready = true;
  return check(tx_test(), "TX one frame");
}

static bool print_nets(void)
{
  static const char *label[] = {"--", "OK", "FAIL", "??"};
  say("");
  say("== CONNECTIONS");
  bool ok = true;
  for (uint8_t i = 0; i < N_COUNT; i++) {
    sayf("  %-10s %-4s %s", nets[i].name, label[nets[i].v], nets[i].hint);
    ok &= nets[i].v == V_OK;
  }
  return ok;
}

static void run_all(void)
{
  for (uint8_t i = 0; i < N_COUNT; i++) {
    nets[i].v = V_SKIP;
    snprintf(nets[i].hint, sizeof(nets[i].hint), "not tested (earlier step failed)");
  }
  driver_ready = false;

  say("");
  say(APP_NAME);
  say(dw3000_clock_from_hse ? "CLOCK HSE 72 MHz" : "CLOCK HSI 48 MHz (crystal failed: check Y1)");
  uint32_t mv = mcu_vdd_mv();
  sayf("STM32 VDD %lu.%02lu V", (unsigned long)(mv / 1000), (unsigned long)(mv % 1000 / 10));

  idle_lines();
  short_test();
  spiBegin(PIN_IRQ, PIN_RST);
  spiSelect(PIN_CS);
  bool spi = cs_miso_test() && dev_id_test();
  if (spi && mosi_test()) {
    rst_test();
    irq_test();
    supply_test();
  }
  bool nets_ok = print_nets();

  say("");
  say("== DRIVER");
  bool drv = spi && driver_tests();

  say("");
  say(nets_ok && drv ? "== RESULT: BOARD OK" : "== RESULT: BOARD FAILED");
  say("commands: r = rerun, t = hold lines for a multimeter, s = report on/off");
}

static void hold_test(void)
{
  struct step_t {
    uint8_t pin;
    uint8_t level;
    const char *text;
  };
  static const step_t steps[] = {
    {PIN_CS, LOW, "CS low: module CSn = 0 V"},
    {PIN_SCK, HIGH, "SCK high: module CLK = 3.3 V"},
    {PIN_MOSI, HIGH, "MOSI high: module MOSI = 3.3 V"},
    {PIN_RST, LOW, "RSTn low: module RSTn = 0 V"},
  };
  say("-- hold test: black probe on module GND, red on the module pin, 5 s each");
  driver_ready = false;
  for (const step_t &s : steps) {
    idle_lines();
    pinMode(s.pin, OUTPUT);
    digitalWrite(s.pin, s.level);
    say(s.text);
    delay(5000);
  }
  idle_lines();
  say("released: CS 3.3 V, SCK 0 V, MOSI 0 V, RSTn 3.3 V. r to rerun");
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

  /* Rerun when a terminal opens the port (upload drops it). */
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
      hold_test();
    } else if (c == 's' || c == 'S') {
      report_on = !report_on;
      say(report_on ? "report on" : "report off");
    }
  }

  if (report_on && driver_ready && millis() - last >= 2000) {
    last = millis();
    uint32_t id = dwt_readdevid();
    sayf("DEV_ID %08lX %s", (unsigned long)id, id_ok(id) ? "OK" : "BAD");
    report_temp();
  }
}
