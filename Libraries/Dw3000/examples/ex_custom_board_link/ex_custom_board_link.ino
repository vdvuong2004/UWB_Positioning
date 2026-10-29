#include "dw3000.h"
#include <math.h>

#define APP_NAME "CUSTOM BOARD LINK TEST v1.0"
/*
 * Radio link check between two custom STM32F103RCT6 + DWM3000 boards, logged
 * over the board's own USB port. Flash it on both boards; set one to TX and
 * the other to RX from the terminal.
 * Board: Generic STM32F1 series, part "Generic F103RCTx",
 * Tools > USB support: "CDC (generic 'Serial' supersede U(S)ART)".
 * Same radio as the ranging sketches: channel 5, PLEN 128, IEEE SFD, 6.8 Mb/s.
 *
 * USB commands:
 *   t = TX mode: a numbered beacon every 100 ms
 *   r = RX mode: every frame with sequence gaps and RX level in dBm
 *   p = next TX power (0x48 used by the peers, 0xC0, 0xFD max)
 *   s = SPI test with the 2-byte header that most registers use
 * Once a second RX prints "SUM good / lost / err" so the loss rate is visible.
 * A healthy link at 0.5 m shows no losses and roughly -60 to -80 dBm.
 */

#if !defined(ARDUINO_ARCH_STM32) || !defined(STM32F103xE)
#error "Select Generic STM32F1 series, part Generic F103RCTx."
#endif
#if !defined(USBCON) || !defined(USBD_USE_CDC)
#error "Tools > USB support: CDC (generic 'Serial' supersede U(S)ART)."
#endif

const uint8_t PIN_RST = DW3000_PIN_RST;
const uint8_t PIN_IRQ = DW3000_PIN_IRQ;
const uint8_t PIN_SS = DW3000_PIN_CS;

static dwt_config_t config = {
    5, DWT_PLEN_128, DWT_PAC8, 9, 9,
    0, /* IEEE 8-symbol SFD */
    DWT_BR_6M8, DWT_PHRMODE_STD, DWT_PHRRATE_STD,
    (129 + 8 - 8), DWT_STS_MODE_OFF, DWT_STS_LEN_64, DWT_PDOA_M0
};

static const uint32_t tx_powers[] = {0x48484848, 0xC0C0C0C0, 0xFDFDFDFD};
#define N_TX_POWERS (sizeof(tx_powers) / sizeof(tx_powers[0]))
static uint8_t power_idx = 0;
static dwt_txconfig_t txconfig = {0x34, 0x48484848, 0x0};

#define TX_PERIOD_MS 100

/* Blink frame: 0xC5, 2-byte sequence, "LINK", then 2 CRC bytes. */
static uint8_t tx_msg[] = {0xC5, 0, 0, 'L', 'I', 'N', 'K'};
#define RX_BUF_LEN 32
static uint8_t rx_buffer[RX_BUF_LEN];

enum link_mode { MODE_RX, MODE_TX };
static link_mode mode = MODE_RX;

static uint16_t tx_seq = 0;
static uint32_t n_good = 0;
static uint32_t n_lost = 0;
static uint32_t n_err = 0;
static uint32_t n_foreign = 0;
static bool have_seq = false;
static uint16_t last_seq = 0;

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

static void halt(const char *msg)
{
  say(msg);
  while (1) {
    delay(1000);
  }
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

static void set_power(uint8_t idx)
{
  power_idx = idx % N_TX_POWERS;
  txconfig.power = tx_powers[power_idx];
  dwt_configuretxrf(&txconfig);
}

static void show_mode(void)
{
  if (mode == MODE_TX) {
    sayf("MODE TX, power %08lX", (unsigned long)tx_powers[power_idx]);
  } else {
    say("MODE RX");
  }
}

static void banner(void)
{
  say(APP_NAME);
  say(dw3000_clock_from_hse ? "CLOCK HSE 72 MHz" : "CLOCK HSI 48 MHz (crystal failed)");
  say("commands: t = TX, r = RX, p = next TX power, s = SPI test");
  show_mode();
}

void setup()
{
  UART_init();
  wait_usb_host();
  banner();

  spiBegin(PIN_IRQ, PIN_RST);
  spiSelect(PIN_SS);
  delay(5);

  /* Bit-banged SPI can miss IDLE_RC on the first reads; a valid ID is enough. */
  bool idle = false;
  uint32_t dev_id = 0;
  for (int attempt = 0; attempt < 50; attempt++) {
    dev_id = dwt_readdevid();
    if (dwt_checkidlerc() || dev_id == DWT_C0_DEV_ID || dev_id == DWT_C0_PDOA_DEV_ID) {
      idle = true;
      break;
    }
    delay(2);
  }
  if (!idle) {
    halt("IDLE FAILED");
  }
  sayf("DEVICE ID %08lX", (unsigned long)dev_id);
  if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
    halt("INIT FAILED");
  }
  if (dwt_configure(&config)) {
    halt("CONFIG FAILED");
  }
  set_power(0);
  spiSetFastRate();
  dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);
  /* Needed for ipatovPower / ipatovAccumCount, the RX level inputs. */
  dwt_configciadiag(DW_CIA_DIAG_LOG_ALL);
}

/* DEV_ID uses a 1-byte SPI header; most registers and the RX buffer need a
 * 2-byte one. Write and read back TX_ANTD (file 1, offset 4) to test that path. */
static void spi_long_header_test(void)
{
  const uint16_t loops = 1000;
  uint16_t bad = 0;
  uint16_t first_wr = 0, first_rd = 0;
  uint16_t v = 0x1234;
  for (uint16_t i = 0; i < loops; i++) {
    v = (uint16_t)(v * 25173u + 13849u);
    dwt_write16bitoffsetreg(TX_ANTD_ID, 0, v);
    uint16_t r = dwt_read16bitoffsetreg(TX_ANTD_ID, 0);
    if (r != v) {
      if (bad == 0) {
        first_wr = v;
        first_rd = r;
      }
      bad++;
    }
  }
  sayf("SPI 2-byte header: %u write/read, %u bad", loops, bad);
  if (bad) {
    sayf("  first bad: wrote %04X read %04X", first_wr, first_rd);
  }
  sayf("  RX_FINFO raw %08lX", (unsigned long)dwt_read32bitreg(RX_FINFO_ID));
  dwt_settxantennadelay(16385);
}

static void handle_usb(void)
{
  static bool host_open = SerialUSB.dtr();

  /* Upload resets the board and drops the COM port. Show the banner again
   * whenever a terminal (re)opens it. */
  bool dtr = SerialUSB.dtr();
  if (dtr && !host_open) {
    delay(100);
    banner();
  }
  host_open = dtr;

  while (SerialUSB.available()) {
    char c = SerialUSB.read();
    if (c == 't' || c == 'T') {
      mode = MODE_TX;
      show_mode();
    } else if (c == 'r' || c == 'R') {
      mode = MODE_RX;
      n_good = n_lost = n_err = n_foreign = 0;
      have_seq = false;
      show_mode();
    } else if (c == 'p' || c == 'P') {
      set_power(power_idx + 1);
      sayf("TX power %08lX", (unsigned long)tx_powers[power_idx]);
    } else if (c == 's' || c == 'S') {
      spi_long_header_test();
    }
  }
}

static void tx_once(void)
{
  static uint32_t last = 0;
  if (millis() - last < TX_PERIOD_MS) {
    return;
  }
  last = millis();

  tx_msg[1] = (uint8_t)(tx_seq & 0xFF);
  tx_msg[2] = (uint8_t)(tx_seq >> 8);
  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_TX);
  dwt_writetxdata(sizeof(tx_msg), tx_msg, 0);
  dwt_writetxfctrl(sizeof(tx_msg) + FCS_LEN, 0, 0);
  dwt_starttx(DWT_START_TX_IMMEDIATE);

  uint32_t start = millis();
  while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK)) {
    if (millis() - start > 20) {
      sayf("TX %u TIMEOUT", tx_seq);
      dwt_writefastCMD(CMD_TXRXOFF);
      tx_seq++;
      return;
    }
  }
  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
  if ((tx_seq % 10) == 0) {
    sayf("TX %u", tx_seq);
  }
  tx_seq++;
}

/* User manual RX level: 10 log10(C * 2^21 / N^2) - A, A = 121.7 at 64 MHz PRF. */
static int rx_level_tenths(void)
{
  dwt_rxdiag_t diag;
  dwt_readdiagnostics(&diag);
  if (diag.ipatovAccumCount == 0 || diag.ipatovPower == 0) {
    return -9999;
  }
  float n = (float)diag.ipatovAccumCount;
  float level = 10.0f * log10f((float)diag.ipatovPower * 2097152.0f / (n * n)) - 121.7f;
  return (int)(level * 10.0f + (level < 0 ? -0.5f : 0.5f));
}

static void rx_summary(void)
{
  static uint32_t last = 0;
  if (millis() - last < 1000) {
    return;
  }
  last = millis();
  sayf("SUM good %lu  lost %lu  err %lu  other %lu", (unsigned long)n_good,
       (unsigned long)n_lost, (unsigned long)n_err, (unsigned long)n_foreign);
}

static void rx_once(void)
{
  /* A leftover bit would end the wait below before any frame arrives. */
  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_GOOD | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_TO);
  dwt_rxenable(DWT_START_RX_IMMEDIATE);

  uint32_t status = 0;
  uint32_t start = millis();
  while (1) {
    /* Read twice and drop bit-banged SPI glitch reads. */
    uint32_t a = dwt_read32bitreg(SYS_STATUS_ID);
    uint32_t b = dwt_read32bitreg(SYS_STATUS_ID);
    if (a == b && (a & 0xC0000000UL) == 0 && ((a >> 16) & 0xFFFFUL) != 0xCA03UL) {
      status = a;
      /* CIAERR alone can come before the CRC result. */
      if (status & (SYS_STATUS_RXFCG_BIT_MASK | (SYS_STATUS_ALL_RX_ERR & ~SYS_STATUS_CIAERR_BIT_MASK))) {
        break;
      }
    }
    if (millis() - start > 200) {
      dwt_writefastCMD(CMD_TXRXOFF);
      return;
    }
  }

  if (!(status & SYS_STATUS_RXFCG_BIT_MASK)) {
    n_err++;
    return;
  }

  uint32_t frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
  if (frame_len > sizeof(rx_buffer)) {
    frame_len = sizeof(rx_buffer);
  }
  dwt_readrxdata(rx_buffer, frame_len, 0);
  if (frame_len != sizeof(tx_msg) + FCS_LEN || rx_buffer[0] != 0xC5 || memcmp(&rx_buffer[3], "LINK", 4) != 0) {
    /* Frames from other sketches (ranging polls) still show the link level. */
    n_foreign++;
    int lvl = rx_level_tenths();
    int mag = lvl < 0 ? -lvl : lvl;
    char line[96];
    int len = snprintf(line, sizeof(line), "OTHER finfo %08lX level %c%d.%d dBm:",
                       (unsigned long)dwt_read32bitreg(RX_FINFO_ID), lvl < 0 ? '-' : ' ', mag / 10, mag % 10);
    for (uint8_t i = 0; i < 12 && i < frame_len && len < (int)sizeof(line); i++) {
      len += snprintf(line + len, sizeof(line) - len, " %02X", rx_buffer[i]);
    }
    say(line);
    return;
  }

  uint16_t seq = (uint16_t)rx_buffer[1] | ((uint16_t)rx_buffer[2] << 8);
  uint16_t gap = 0;
  if (have_seq) {
    gap = (uint16_t)(seq - last_seq - 1);
    /* A big jump is the TX board restarting, not real loss. */
    if (gap < 1000) {
      n_lost += gap;
    }
  }
  have_seq = true;
  last_seq = seq;
  n_good++;

  int lvl = rx_level_tenths();
  char sign = lvl < 0 ? '-' : ' ';
  int mag = lvl < 0 ? -lvl : lvl;
  sayf("RX %u%s  level %c%d.%d dBm%s", seq, gap && gap < 1000 ? " (gap)" : "", sign, mag / 10, mag % 10,
       (status & SYS_STATUS_CIAERR_BIT_MASK) ? "  CIAERR" : "");
}

void loop()
{
  handle_usb();
  if (mode == MODE_TX) {
    tx_once();
  } else {
    rx_once();
    rx_summary();
  }
}
