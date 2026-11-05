#include "dw3000.h"
#include <math.h>

#define APP_NAME "UWB TAG v2.0"
/*
 * UWB tag (SS-TWR initiator): custom STM32F103RCT6 board + DWM3000.
 * Polls every anchor in ANCHORS[] in turn and prints one line per range:
 *     DIST <anchor id> <metres> m
 * Tools/uwb_tracker reads these lines, filters them and draws the 2D map.
 *
 * Arduino IDE: Board "Generic STM32F1 series", part "Generic F103RCTx",
 * Tools > USB support: "CDC (generic 'Serial' supersede U(S)ART)".
 * Log and commands on the board's USB port (USART1 PA9/PA10 gets the same text).
 *
 * USB commands: p = pause/resume, z = zero stats, 1..9 = turn that anchor on/off.
 *
 * Protocol (same in uwb_anchor_dw3000 and uwb_anchor_dw1000, change all three together):
 *   radio  channel 5, PLEN 128, PAC 8, code 9, IEEE 8-symbol SFD, 6.8 Mb/s, STS off
 *   poll   41 88 seq CA DE dst(anchor) src(tag) E0                       + CRC
 *   resp   41 88 seq CA DE dst(tag) src(anchor) E1 poll_rx_ts resp_tx_ts + CRC
 *   Addresses are 16-bit little-endian. The anchor copies seq and answers
 *   REPLY_DELAY_US after the poll's RX timestamp.
 */

#if !defined(ARDUINO_ARCH_STM32) || !defined(STM32F103xE)
#error "Select Generic STM32F1 series, part Generic F103RCTx."
#endif
#if !defined(USBCON) || !defined(USBD_USE_CDC)
#error "Tools > USB support: CDC (generic 'Serial' supersede U(S)ART)."
#endif

#define TAG_ID 100

/* TX power per anchor. 0xC0 saturates a DWM3000 that is close by, so it
 * never decodes the poll; the DWM1000 needs the stronger poll. */
#define PWR_DW3000 0x48
#define PWR_DW1000 0xC0

struct anchor_t {
  uint16_t id;
  uint8_t tx_power;
};
static const anchor_t ANCHORS[] = {
  {1, PWR_DW3000},
  {2, PWR_DW3000},
  {3, PWR_DW1000},
};
#define N_ANCHORS (sizeof(ANCHORS) / sizeof(ANCHORS[0]))

/* One round polls every anchor; then wait so a round takes this long. */
#define ROUND_PERIOD_MS 100
#define STATS_EVERY_ROUNDS 50

#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385

/* Anchors answer 3 ms after the poll. Open RX 2 ms after the poll and keep
 * it open long enough for any anchor to answer. */
#define REPLY_DELAY_US 3000
#define POLL_TX_TO_RESP_RX_DLY_UUS 2000
#define RESP_RX_TIMEOUT_UUS 10000
#define RESP_WAIT_MS 30

static dwt_config_t config = {
    5, DWT_PLEN_128, DWT_PAC8, 9, 9,
    0, /* IEEE 8-symbol SFD: required for the DWM1000 at 6.8 Mb/s. */
    DWT_BR_6M8, DWT_PHRMODE_STD, DWT_PHRRATE_STD,
    (129 + 8 - 8), DWT_STS_MODE_OFF, DWT_STS_LEN_64, DWT_PDOA_M0
};

#define FC_LO 0x41
#define FC_HI 0x88
#define PAN_LO 0xCA
#define PAN_HI 0xDE
#define FUNC_POLL 0xE0
#define FUNC_RESP 0xE1
#define IDX_SEQ 2
#define IDX_DST 5
#define IDX_SRC 7
#define IDX_FUNC 9
#define IDX_POLL_RX_TS 10
#define IDX_RESP_TX_TS 14
/* Payload plus 2 CRC bytes the chip fills in. */
#define POLL_LEN 12
#define RESP_LEN 20

static uint8_t poll_msg[POLL_LEN] = {FC_LO, FC_HI, 0, PAN_LO, PAN_HI, 0, 0,
                                     TAG_ID & 0xFF, TAG_ID >> 8, FUNC_POLL, 0, 0};
static uint8_t rx_buffer[24];
static uint8_t seq = 0;

struct anchor_stats_t {
  bool enabled;
  uint16_t polls;
  uint16_t replies;
  /* Spread over the current stats window, in cm. */
  uint16_t n;
  float sum;
  float sum_sq;
  int min_cm;
  int max_cm;
};
static anchor_stats_t stats[N_ANCHORS];
static uint16_t rounds = 0;
static uint16_t n_recover = 0;
static bool paused = false;

static void say(const char *msg)
{
  test_run_info((unsigned char *)msg);
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

static void print_anchors(void)
{
  char line[64];
  int len = snprintf(line, sizeof(line), "anchors:");
  for (uint8_t i = 0; i < N_ANCHORS && len < (int)sizeof(line); i++) {
    if (stats[i].enabled) {
      len += snprintf(line + len, sizeof(line) - len, " %u", ANCHORS[i].id);
    }
  }
  say(line);
}

static void banner(void)
{
  char line[40];
  say(APP_NAME);
  say(dw3000_clock_from_hse ? "CLOCK HSE 72 MHz" : "CLOCK HSI 48 MHz (crystal failed)");
  snprintf(line, sizeof(line), "tag id %u", TAG_ID);
  say(line);
  print_anchors();
  say("commands: p = pause/resume, z = zero stats, 1..9 = anchor on/off");
}

/* spiSelect() pulses RSTn, so this is also the recovery path. */
static bool dw_start(void)
{
  spiSelect(DW3000_PIN_CS);
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
  char line[40];
  if (!idle) {
    snprintf(line, sizeof(line), "IDLE FAILED id %08lX", (unsigned long)dev_id);
    say(line);
    return false;
  }
  snprintf(line, sizeof(line), "DEVICE ID %08lX", (unsigned long)dev_id);
  say(line);
  if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
    say("INIT FAILED");
    return false;
  }
  /* A failed RX calibration still leaves TX usable, so keep polling. */
  if (dwt_configure(&config)) {
    say("CONFIG FAILED (RX cal), polls still go out");
  }
  spiSetFastRate();
  dwt_setrxantennadelay(RX_ANT_DLY);
  dwt_settxantennadelay(TX_ANT_DLY);
  dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
  dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
  dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);
  return true;
}

void setup()
{
  UART_init();
  for (uint8_t i = 0; i < N_ANCHORS; i++) {
    stats[i].enabled = true;
  }
  wait_usb_host();
  banner();

  spiBegin(DW3000_PIN_IRQ, DW3000_PIN_RST);
  if (!dw_start()) {
    halt("DW3000 START FAILED");
  }
  say("ranging");
}

/* Reads SYS_STATUS twice and drops bit-banged SPI glitch reads. */
static bool read_status(uint32_t *out)
{
  uint32_t a = dwt_read32bitreg(SYS_STATUS_ID);
  uint32_t b = dwt_read32bitreg(SYS_STATUS_ID);
  if (a != b || (a & 0xC0000000UL) != 0 || ((a >> 16) & 0xFFFFUL) == 0xCA03UL) {
    return false;
  }
  *out = a;
  return true;
}

static void record(anchor_stats_t *st, int cm)
{
  st->replies++;
  if (st->n == 0 || cm < st->min_cm) {
    st->min_cm = cm;
  }
  if (st->n == 0 || cm > st->max_cm) {
    st->max_cm = cm;
  }
  st->n++;
  st->sum += cm;
  st->sum_sq += (float)cm * cm;
}

static void range_one(uint8_t idx)
{
  const anchor_t *a = &ANCHORS[idx];
  anchor_stats_t *st = &stats[idx];
  dwt_txconfig_t tx = {0x34, (uint32_t)a->tx_power * 0x01010101UL, 0x0};
  dwt_configuretxrf(&tx);

  poll_msg[IDX_SEQ] = seq;
  poll_msg[IDX_DST] = a->id & 0xFF;
  poll_msg[IDX_DST + 1] = a->id >> 8;
  /* A leftover RX timeout bit would abort the next poll before it is sent. */
  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK | SYS_STATUS_ALL_RX_GOOD |
                                   SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
  dwt_writetxdata(POLL_LEN, poll_msg, 0);
  dwt_writetxfctrl(POLL_LEN, 0, 1);
  dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
  st->polls++;

  uint32_t status = 0;
  uint32_t wait_start = millis();
  bool tx_done = false;
  while ((millis() - wait_start) <= RESP_WAIT_MS) {
    if (!read_status(&status)) {
      continue;
    }
    if (status & SYS_STATUS_TXFRS_BIT_MASK) {
      tx_done = true;
    }
    if (status & SYS_STATUS_RXFCG_BIT_MASK) {
      break;
    }
    /* Ignore a timeout that belongs to the previous exchange. CIAERR alone
     * can come before the CRC result, so it does not end the wait. */
    if (tx_done && (status & (SYS_STATUS_ALL_RX_TO | (SYS_STATUS_ALL_RX_ERR & ~SYS_STATUS_CIAERR_BIT_MASK)))) {
      break;
    }
  }
  uint8_t poll_seq = seq++;

  char line[48];
  if (!(status & SYS_STATUS_RXFCG_BIT_MASK)) {
    dwt_writefastCMD(CMD_TXRXOFF);
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
    snprintf(line, sizeof(line), "NO RESP %u %08lX", a->id, (unsigned long)status);
    say(line);
    return;
  }
  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
  uint32_t frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
  bool ok = frame_len == RESP_LEN;
  if (ok) {
    dwt_readrxdata(rx_buffer, RESP_LEN, 0);
    ok = rx_buffer[0] == FC_LO && rx_buffer[1] == FC_HI && rx_buffer[IDX_SEQ] == poll_seq &&
         rx_buffer[3] == PAN_LO && rx_buffer[4] == PAN_HI &&
         rx_buffer[IDX_DST] == (TAG_ID & 0xFF) && rx_buffer[IDX_DST + 1] == (TAG_ID >> 8) &&
         rx_buffer[IDX_SRC] == (a->id & 0xFF) && rx_buffer[IDX_SRC + 1] == (a->id >> 8) &&
         rx_buffer[IDX_FUNC] == FUNC_RESP;
  }
  if (!ok) {
    snprintf(line, sizeof(line), "BAD RESP %u len %lu", a->id, (unsigned long)frame_len);
    say(line);
    return;
  }

  uint32_t poll_tx_ts = dwt_readtxtimestamplo32();
  uint32_t resp_rx_ts = dwt_readrxtimestamplo32();
  /* Read twice: a glitch read would bend the clock correction. */
  int16_t clk_a = dwt_readclockoffset();
  int16_t clk_b = dwt_readclockoffset();
  double clock_offset_ratio = (clk_a == clk_b) ? (double)clk_a / (double)(1UL << 26) : 0.0;
  uint32_t poll_rx_ts, resp_tx_ts;
  resp_msg_get_ts(&rx_buffer[IDX_POLL_RX_TS], &poll_rx_ts);
  resp_msg_get_ts(&rx_buffer[IDX_RESP_TX_TS], &resp_tx_ts);

  int32_t rtd_init = (int32_t)(resp_rx_ts - poll_tx_ts);
  int32_t rtd_resp = (int32_t)(resp_tx_ts - poll_rx_ts);
  double tof = (((double)rtd_init - (double)rtd_resp * (1.0 - clock_offset_ratio)) / 2.0) * DWT_TIME_UNITS;
  double distance = tof * SPEED_OF_LIGHT;

  /* newlib-nano printf has no %f. */
  int cm = (int)(distance * 100.0 + (distance < 0 ? -0.5 : 0.5));
  record(st, cm);
  int abs_cm = cm < 0 ? -cm : cm;
  snprintf(line, sizeof(line), "DIST %u %s%d.%02d m", a->id, cm < 0 ? "-" : "", abs_cm / 100, abs_cm % 100);
  say(line);
}

static void print_stats(void)
{
  char line[96];
  int len = snprintf(line, sizeof(line), "STATS");
  for (uint8_t i = 0; i < N_ANCHORS && len < (int)sizeof(line); i++) {
    len += snprintf(line + len, sizeof(line) - len, "  %u %u/%u", ANCHORS[i].id, stats[i].replies, stats[i].polls);
  }
  if (len < (int)sizeof(line)) {
    snprintf(line + len, sizeof(line) - len, "  recover %u", n_recover);
  }
  say(line);

  for (uint8_t i = 0; i < N_ANCHORS; i++) {
    anchor_stats_t *st = &stats[i];
    if (st->n == 0) {
      continue;
    }
    float mean = st->sum / st->n;
    float var = st->sum_sq / st->n - mean * mean;
    int mean_cm = (int)(mean + (mean < 0 ? -0.5f : 0.5f));
    int std_cm = (int)(sqrtf(var > 0 ? var : 0) + 0.5f);
    snprintf(line, sizeof(line), "SPREAD %u n %u  mean %d cm  std %d cm  min %d  max %d",
             ANCHORS[i].id, st->n, mean_cm, std_cm, st->min_cm, st->max_cm);
    say(line);
    st->n = 0;
    st->sum = st->sum_sq = 0;
  }
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
    if (c == 'p' || c == 'P') {
      paused = !paused;
      say(paused ? "paused" : "ranging");
    } else if (c == 'z' || c == 'Z') {
      for (uint8_t i = 0; i < N_ANCHORS; i++) {
        stats[i].polls = stats[i].replies = 0;
      }
      rounds = 0;
      n_recover = 0;
      say("stats zeroed");
    } else if (c >= '1' && c <= '9') {
      for (uint8_t i = 0; i < N_ANCHORS; i++) {
        if (ANCHORS[i].id == (uint16_t)(c - '0')) {
          stats[i].enabled = !stats[i].enabled;
        }
      }
      print_anchors();
    }
  }
}

void loop()
{
  handle_usb();
  if (paused) {
    return;
  }
  uint32_t round_start = millis();

  uint32_t id = dwt_readdevid();
  if (id != DWT_C0_DEV_ID && id != DWT_C0_PDOA_DEV_ID) {
    char line[40];
    snprintf(line, sizeof(line), "DW RECOVER (id %08lX)", (unsigned long)id);
    say(line);
    n_recover++;
    dw_start();
  }

  for (uint8_t i = 0; i < N_ANCHORS; i++) {
    if (stats[i].enabled) {
      range_one(i);
    }
  }

  if (++rounds >= STATS_EVERY_ROUNDS) {
    print_stats();
    rounds = 0;
  }
  while ((millis() - round_start) < ROUND_PERIOD_MS) {
    handle_usb();
  }
}
