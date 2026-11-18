#include "dw3000.h"
#include <math.h>
#include <stdlib.h>

#define APP_NAME "UWB TAG v3.2"
/*
 * UWB tag (SS-TWR initiator): custom STM32F103RCT6 board + DWM3000.
 * Fixed frame (ROUND_PERIOD_MS, 25 ms = 40 Hz):
 *   1. range every anchor in ANCHORS[] (a miss is retried only while the frame has time),
 *   2. compensate:  d_comp = scale * d_raw - bias   (per anchor, from calibration),
 *   3. median of the last MEDIAN_N compensated ranges (removes spikes and mode jumps),
 *   4. IIR low-pass: d_filt[n] = alpha * d_med[n] + (1 - alpha) * d_filt[n-1], with a jump gate,
 *   5. dead band: the output moves only when d_filt leaves +-DEADBAND_M around it,
 *      so a tag standing still gives a steady value,
 *   6. print one line per anchor ranged in this frame:
 *        DIST <anchor id> <d_filt> m raw <d_raw>
 * TX power adapts per anchor (see PWR_LADDER); "PWR <id> <hex>" marks a change.
 * Tools/uwb_tracker reads these lines and draws the 2D map.
 *
 * Arduino IDE: Board "Generic STM32F1 series", part "Generic F103RCTx",
 * Tools > USB support: "CDC (generic 'Serial' supersede U(S)ART)".
 * Log and commands on the board's USB port only: USART1 printing waits for every
 * character (~5 ms per line) and does not fit the 25 ms frame.
 *
 * USB commands: p = pause/resume, z = zero stats, 1..9 = turn that anchor on/off,
 *               f = next round period (25/100/50/20/10/0 ms) to find the max update rate,
 *               d = DIST lines on/off, r = tries per anchor per round (2/3/1),
 *               "C<id> <scale> <bias>\n" = range compensation of that anchor (bias in m),
 *               "L<alpha>\n" = IIR coefficient, 0 < alpha <= 1 (1 = no filtering).
 *               uwb_tracker sends C and L from anchors.json when it connects.
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

/* Adaptive TX power per anchor. A strong poll saturates a DWM3000 close by (it
 * never decodes), a weak one is lost far away, so no fixed level fits a room.
 * Each anchor starts at its level in PWR_LADDER and:
 *  - a missed reply steps one level up, wrapping from the top back to the
 *    bottom (a saturated anchor also just stays silent);
 *  - PWR_STRONG_ROW replies in a row above PWR_DOWN_DBM step one level down.
 *    If the next reply after a step down is missed, that anchor waits twice as
 *    long before the next try down (up to PWR_HOLD_MAX).
 * The DW3000 anchor picks its own reply power from the poll level. */
static const uint8_t PWR_LADDER[] = {0x48, 0x80, 0xC0, 0xFD};
#define N_PWR (sizeof(PWR_LADDER))
#define LVL_DW3000 0 /* 0x48 */
#define LVL_DW1000 2 /* 0xC0: the DWM1000 needs the stronger poll */
#define PWR_DOWN_DBM (-75.0f)
#define PWR_STRONG_ROW 20
#define PWR_HOLD_MAX 640

struct anchor_t {
  uint16_t id;
  uint8_t start_lvl;
};
static const anchor_t ANCHORS[] = {
  {1, LVL_DW3000},
  {2, LVL_DW1000},
  {3, LVL_DW3000},
};
#define N_ANCHORS (sizeof(ANCHORS) / sizeof(ANCHORS[0]))

/* Range compensation per ANCHORS[] entry: d_comp = scale * d_raw - bias.
 * bias = measured - true at a known distance (antenna delay error); scale fixes a
 * slope if a multi-distance calibration shows one. The tracker sends anchors.json values. */
struct comp_t {
  float scale, bias;
};
static comp_t comp[N_ANCHORS] = {
  {1.0f, 0.0f},
  {1.0f, 0.0f},
  {1.0f, 0.0f},
};

/* First-order IIR low-pass on each compensated range, per anchor x:
 *   d_x[n] = alpha * d_x,meas[n] + (1 - alpha) * d_x[n-1]
 * At 25 ms frames alpha 0.1 cuts the noise std to ~0.23x (sqrt(a / (2 - a)))
 * with ~225 ms lag (25 ms * (1 - a) / a). */
#define IIR_ALPHA 0.1f
static float iir_alpha = IIR_ALPHA;
/* A range that differs from the filter by more than JUMP_M in one frame (20 m/s)
 * is a multipath spike: dropped. JUMP_RESET in a row means the tag really moved
 * (or the filter is wrong): restart from the new value. After IIR_STALE_MS without
 * a range the old value is meaningless, so the next range restarts the filter too. */
#define JUMP_M 0.5f
#define JUMP_RESET 4
#define IIR_STALE_MS 500
/* Median before the IIR: an IIR smears a 30 cm spike over many frames, a median
 * of 5 drops it (and a NLOS mode jump that lasts < 3 frames) for ~2 frames of lag. */
#define MEDIAN_N 5
/* Dead band after the IIR: the filtered value still wanders ~1 cm per frame when
 * the tag stands still. Moving, the output trails the filter by at most this. */
#define DEADBAND_M 0.02f

/* One round polls every anchor; then wait so a round takes this long.
 * 'f' steps through ROUND_PERIODS; 0 = poll back to back. */
#define ROUND_PERIOD_MS 25
static const uint16_t ROUND_PERIODS[] = {25, 100, 50, 20, 10, 0};
#define STATS_PERIOD_MS 5000

#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385

/* Anchors answer exactly 3 ms after the poll RX timestamp (DW1000 anchor since
 * v2.2). RX opens 2 ms after the poll; the reply is over ~1.1 ms later. 4 ms keeps
 * margin (also for an old DW1000 anchor, ~1.5 ms later) and a miss costs ~6 ms
 * instead of 12, so a retry fits in the round. */
#define REPLY_DELAY_US 3000
#define POLL_TX_TO_RESP_RX_DLY_UUS 2000
#define RESP_RX_TIMEOUT_UUS 4000
#define RESP_WAIT_MS 10

/* A missed reply is retried at once with a new seq, so one lost frame no longer
 * costs the anchor's range for the whole round. 'r' cycles TRIES_CHOICES. */
#define MAX_TRIES 2
/* Frame budget for retries: a missed exchange (2 ms + 4 ms RX window + setup)
 * and the compensation/IIR/print step after the last one. */
#define EXCHANGE_MAX_US 6500
#define FRAME_TAIL_US 1000
static const uint8_t TRIES_CHOICES[] = {2, 3, 1};
static uint8_t max_tries = MAX_TRIES;

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
  uint16_t asked;   /* rounds this anchor was polled in */
  uint16_t ranged;  /* ... and gave a range, retries included */
  /* Spread over the current stats window, in cm. */
  uint16_t n;
  float sum;
  float sum_sq;
  int min_cm;
  int max_cm;
  float rx_dbm_sum; /* reply RX level, this window */
  float fsum, fsum_sq; /* filtered range (cm), this window, n samples */
  /* IIR state (kept across windows). */
  float y;
  bool y_ok;
  uint8_t jump_row;
  uint32_t y_ms;
  float med_buf[MEDIAN_N]; /* last compensated ranges, ring buffer */
  uint8_t med_n, med_i;
  float out; /* dead-band output, what DIST prints */
  /* Adaptive power state (kept across windows). */
  uint8_t lvl;
  uint8_t strong_row;
  uint16_t hold;
  bool just_down;
};
static anchor_stats_t stats[N_ANCHORS];
static uint16_t round_period_ms = ROUND_PERIOD_MS;
static uint16_t rounds = 0;
static uint32_t stats_start_ms = 0;
/* Time of one good exchange (poll TX to distance), this stats window. */
static uint32_t xchg_us_sum = 0, xchg_us_max = 0, xchg_n = 0;
static uint16_t n_recover = 0;
static bool paused = false;
static bool print_dist = true;

/* This frame's raw ranges. */
static bool rng_ok[N_ANCHORS];
static float rng_m[N_ANCHORS];

/* Frame stats, this stats window. */
static uint16_t n_jump = 0, n_jump_reset = 0, n_overrun = 0, n_retry_skip = 0;

/* USB CDC only: the write is copied to a buffer and returns at once. */
static void say(const char *msg)
{
  SerialUSB.write(msg);
  SerialUSB.write("\r\n");
}

/* Appends v with 2 decimals (newlib-nano printf has no %f). Returns chars written. */
static int put_fixed(char *s, int n, float v)
{
  if (n <= 0) {
    return 0;
  }
  long c = lroundf(v * 100.0f);
  unsigned long a = c < 0 ? -c : c;
  int len = snprintf(s, n, "%s%lu.%02lu", c < 0 ? "-" : "", a / 100, a % 100);
  return len < n ? len : n - 1;
}

static void print_comp(uint8_t i)
{
  char line[64];
  int len = snprintf(line, sizeof(line), "anchor %u scale ", ANCHORS[i].id);
  len += put_fixed(line + len, sizeof(line) - len, comp[i].scale);
  len += snprintf(line + len, sizeof(line) - len, " bias ");
  put_fixed(line + len, sizeof(line) - len, comp[i].bias);
  say(line);
}

static void print_alpha(void)
{
  char line[32];
  int len = snprintf(line, sizeof(line), "iir alpha ");
  put_fixed(line + len, sizeof(line) - len, iir_alpha);
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
  for (uint8_t i = 0; i < N_ANCHORS; i++) {
    print_comp(i);
  }
  print_alpha();
  snprintf(line, sizeof(line), "round period %u ms", round_period_ms);
  say(line);
  snprintf(line, sizeof(line), "tries per anchor %u", max_tries);
  say(line);
  say("commands: p = pause/resume, z = zero stats, 1..9 = anchor on/off, f = round period, d = DIST on/off, r = tries");
  /* Last: uwb_tracker answers this line with the C/L commands. */
  print_anchors();
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
  /* The RX level registers (IP_DIAG_1/12) stay 0 unless CIA diagnostics are on. */
  dwt_configciadiag(DW_CIA_DIAG_LOG_ALL);
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
    stats[i].lvl = ANCHORS[i].start_lvl;
    stats[i].hold = PWR_STRONG_ROW;
  }
  wait_usb_host();
  banner();

  spiBegin(DW3000_PIN_IRQ, DW3000_PIN_RST);
  if (!dw_start()) {
    halt("DW3000 START FAILED");
  }
  reset_window();
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

/* Estimated RX level of the frame just received (DW3000 user manual 4.7.2,
 * PRF 64 MHz): 10 log10(C 2^21 / N^2) + 6 D - 121.7 dBm. The estimate reads low
 * when the signal is very strong. */
#define DGC_DBG_ADDR 0x30060 /* not in dw3000_regs.h; DGC decision in bits 30:28 */
static float rx_level_dbm(void)
{
  uint32_t c = dwt_read32bitreg(IP_DIAG_1_ID) & 0x1FFFFUL;
  uint32_t n = dwt_read32bitreg(IP_DIAG_12_ID) & 0xFFFUL;
  uint32_t d = (dwt_read32bitreg(DGC_DBG_ADDR) >> 28) & 0x7UL;
  if (c == 0 || n == 0) {
    return -150.0f;
  }
  return 10.0f * log10f((float)c * 2097152.0f / ((float)n * (float)n)) + 6.0f * d - 121.7f;
}

static float last_rx_dbm = -150.0f; /* level of the last good reply */

static void print_pwr(uint8_t i)
{
  char line[32];
  snprintf(line, sizeof(line), "PWR %u %02X", ANCHORS[i].id, PWR_LADDER[stats[i].lvl]);
  say(line);
}

/* Called after every exchange; rx_dbm only valid when ok. */
static void adapt_power(uint8_t i, bool ok, float rx_dbm)
{
  anchor_stats_t *st = &stats[i];
  if (!ok) {
    st->hold = st->just_down ? (st->hold * 2 > PWR_HOLD_MAX ? PWR_HOLD_MAX : st->hold * 2) : PWR_STRONG_ROW;
    st->just_down = false;
    st->strong_row = 0;
    st->lvl = (st->lvl + 1) % N_PWR;
    print_pwr(i);
    return;
  }
  st->just_down = false;
  if (rx_dbm > PWR_DOWN_DBM && st->lvl > 0) {
    if (++st->strong_row >= st->hold) {
      st->strong_row = 0;
      st->lvl--;
      st->just_down = true;
      print_pwr(i);
    }
  } else {
    st->strong_row = 0;
  }
}

/* One poll/response exchange. Returns true and stores the range on success. */
static bool range_one(uint8_t idx)
{
  const anchor_t *a = &ANCHORS[idx];
  anchor_stats_t *st = &stats[idx];
  uint32_t t0 = micros();
  dwt_txconfig_t tx = {0x34, (uint32_t)PWR_LADDER[st->lvl] * 0x01010101UL, 0x0};
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
    return false;
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
    return false;
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
  last_rx_dbm = rx_level_dbm();
  st->rx_dbm_sum += last_rx_dbm;
  rng_ok[idx] = true;
  rng_m[idx] = (float)distance;
  uint32_t dt = micros() - t0;
  xchg_us_sum += dt;
  xchg_n++;
  if (dt > xchg_us_max) {
    xchg_us_max = dt;
  }
  return true;
}

/* Median of the last med_n compensated ranges of one anchor. */
static float median_push(anchor_stats_t *st, float d)
{
  st->med_buf[st->med_i] = d;
  st->med_i = (st->med_i + 1) % MEDIAN_N;
  if (st->med_n < MEDIAN_N) {
    st->med_n++;
  }
  float v[MEDIAN_N];
  uint8_t n = st->med_n;
  for (uint8_t k = 0; k < n; k++) {
    float x = st->med_buf[k];
    int8_t j = k - 1;
    while (j >= 0 && v[j] > x) {
      v[j + 1] = v[j];
      j--;
    }
    v[j + 1] = x;
  }
  return (n & 1) ? v[n / 2] : 0.5f * (v[n / 2 - 1] + v[n / 2]);
}

/* (Re)starts one anchor's filter chain at d. */
static void filter_restart(anchor_stats_t *st, float d, uint32_t now)
{
  st->med_n = st->med_i = 0;
  median_push(st, d);
  st->y = st->out = d;
  st->y_ok = true;
  st->jump_row = 0;
  st->y_ms = now;
}

/* Steps 2-6 of the frame: compensate, median, IIR, dead band, print. */
static void iir_frame(void)
{
  uint32_t now = millis();
  char line[64];
  for (uint8_t i = 0; i < N_ANCHORS; i++) {
    if (!rng_ok[i]) {
      continue;
    }
    anchor_stats_t *st = &stats[i];
    float d = comp[i].scale * rng_m[i] - comp[i].bias;
    if (!st->y_ok || now - st->y_ms > IIR_STALE_MS) {
      filter_restart(st, d, now);
    } else {
      float m = median_push(st, d);
      if (fabsf(m - st->y) > JUMP_M) {
        if (++st->jump_row >= JUMP_RESET) {
          filter_restart(st, d, now);
          n_jump_reset++;
        } else {
          n_jump++; /* keep the old value, but still report it below */
        }
      } else {
        st->y = iir_alpha * m + (1.0f - iir_alpha) * st->y;
        st->jump_row = 0;
        st->y_ms = now;
      }
      if (st->y - st->out > DEADBAND_M) {
        st->out = st->y - DEADBAND_M;
      } else if (st->out - st->y > DEADBAND_M) {
        st->out = st->y + DEADBAND_M;
      }
    }
    float y_cm = st->out * 100.0f;
    st->fsum += y_cm;
    st->fsum_sq += y_cm * y_cm;
    if (print_dist) {
      int len = snprintf(line, sizeof(line), "DIST %u ", ANCHORS[i].id);
      len += put_fixed(line + len, sizeof(line) - len, st->out);
      len += snprintf(line + len, sizeof(line) - len, " m raw ");
      put_fixed(line + len, sizeof(line) - len, rng_m[i]);
      say(line);
    }
  }
}

/* Starts a new stats window (rates, spread, exchange time). */
static void reset_window(void)
{
  for (uint8_t i = 0; i < N_ANCHORS; i++) {
    stats[i].n = 0;
    stats[i].sum = stats[i].sum_sq = 0;
    stats[i].rx_dbm_sum = 0;
    stats[i].fsum = stats[i].fsum_sq = 0;
  }
  rounds = 0;
  xchg_us_sum = xchg_us_max = xchg_n = 0;
  n_jump = n_jump_reset = n_overrun = n_retry_skip = 0;
  stats_start_ms = millis();
}

static void print_stats(void)
{
  char line[128];
  uint32_t elapsed = millis() - stats_start_ms;
  if (elapsed == 0) {
    elapsed = 1;
  }
  uint32_t round_x10 = rounds ? elapsed * 10 / rounds : 0;
  snprintf(line, sizeof(line), "RATE period %u ms  round %lu.%lu ms  exchange avg %lu us max %lu us",
           round_period_ms, (unsigned long)(round_x10 / 10), (unsigned long)(round_x10 % 10),
           (unsigned long)(xchg_n ? xchg_us_sum / xchg_n : 0), (unsigned long)xchg_us_max);
  say(line);
  snprintf(line, sizeof(line), "IIR jumps dropped %u  restarts %u  overrun %u  retries skipped %u",
           n_jump, n_jump_reset, n_overrun, n_retry_skip);
  say(line);
  int len = snprintf(line, sizeof(line), "STATS");
  for (uint8_t i = 0; i < N_ANCHORS && len < (int)sizeof(line); i++) {
    len += snprintf(line + len, sizeof(line) - len, "  %u %u/%u", ANCHORS[i].id, stats[i].replies, stats[i].polls);
  }
  if (len < (int)sizeof(line)) {
    len += snprintf(line + len, sizeof(line) - len, "  rounds");
  }
  for (uint8_t i = 0; i < N_ANCHORS && len < (int)sizeof(line); i++) {
    len += snprintf(line + len, sizeof(line) - len, " %u/%u", stats[i].ranged, stats[i].asked);
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
    uint32_t hz_x10 = (uint32_t)st->n * 10000UL / elapsed;
    int rx_dbm = (int)lroundf(st->rx_dbm_sum / st->n);
    float fmean = st->fsum / st->n;
    float fvar = st->fsum_sq / st->n - fmean * fmean;
    int fstd_cm = (int)(sqrtf(fvar > 0 ? fvar : 0) + 0.5f);
    snprintf(line, sizeof(line), "SPREAD %u n %u  %lu.%lu Hz  mean %d cm  std %d cm  filt std %d cm  min %d  max %d  rx %d dBm  pwr %02X",
             ANCHORS[i].id, st->n, (unsigned long)(hz_x10 / 10), (unsigned long)(hz_x10 % 10),
             mean_cm, std_cm, fstd_cm, st->min_cm, st->max_cm, rx_dbm, PWR_LADDER[st->lvl]);
    say(line);
  }
  reset_window();
}

/* "C<id> <scale> <bias>" or "L<alpha>". */
static void run_line_cmd(char *cmd)
{
  char *p = cmd + 1;
  if (cmd[0] == 'C') {
    long id = strtol(p, &p, 10);
    float scale = strtof(p, &p);
    float bias = strtof(p, &p);
    for (uint8_t i = 0; i < N_ANCHORS; i++) {
      if (ANCHORS[i].id == id) {
        if (scale > 0.5f && scale < 1.5f) {
          comp[i].scale = scale;
          comp[i].bias = bias;
          stats[i].y_ok = false; /* restart the IIR on the new compensation */
        }
        print_comp(i);
        return;
      }
    }
    say("C: unknown anchor id");
  } else if (cmd[0] == 'L') {
    float a = strtof(p, &p);
    if (a > 0.0f && a <= 1.0f) {
      iir_alpha = a;
    }
    print_alpha();
  }
}

static void handle_usb(void)
{
  static char cmd[48];
  static uint8_t cmd_len = 0;

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
    /* Line commands start with C or L and end at the newline. */
    if (cmd_len > 0) {
      if (c == '\n' || c == '\r') {
        cmd[cmd_len] = 0;
        run_line_cmd(cmd);
        cmd_len = 0;
      } else if (cmd_len < sizeof(cmd) - 1) {
        cmd[cmd_len++] = c;
      }
      continue;
    }
    if (c == 'C' || c == 'L') {
      cmd[0] = c;
      cmd_len = 1;
    } else if (c == 'd' || c == 'D') {
      print_dist = !print_dist;
      say(print_dist ? "DIST lines on" : "DIST lines off");
    } else if (c == 'p' || c == 'P') {
      paused = !paused;
      say(paused ? "paused" : "ranging");
    } else if (c == 'z' || c == 'Z') {
      for (uint8_t i = 0; i < N_ANCHORS; i++) {
        stats[i].polls = stats[i].replies = stats[i].asked = stats[i].ranged = 0;
      }
      n_recover = 0;
      reset_window();
      say("stats zeroed");
    } else if (c == 'f' || c == 'F') {
      uint8_t n = sizeof(ROUND_PERIODS) / sizeof(ROUND_PERIODS[0]);
      uint8_t k = 0;
      while (k < n && ROUND_PERIODS[k] != round_period_ms) {
        k++;
      }
      round_period_ms = ROUND_PERIODS[(k + 1) % n];
      reset_window();
      char line[32];
      snprintf(line, sizeof(line), "round period %u ms", round_period_ms);
      say(line);
    } else if (c == 'r' || c == 'R') {
      uint8_t n = sizeof(TRIES_CHOICES);
      uint8_t k = 0;
      while (k < n && TRIES_CHOICES[k] != max_tries) {
        k++;
      }
      max_tries = TRIES_CHOICES[(k + 1) % n];
      char line[32];
      snprintf(line, sizeof(line), "tries per anchor %u", max_tries);
      say(line);
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
  uint32_t round_start_us = micros();
  for (uint8_t i = 0; i < N_ANCHORS; i++) {
    rng_ok[i] = false;
  }

  /* Re-read before resetting: one bit-banged glitch read would cost a whole
   * re-init (and the round) for nothing. */
  uint32_t id = dwt_readdevid();
  if (id != DWT_C0_DEV_ID && id != DWT_C0_PDOA_DEV_ID) {
    id = dwt_readdevid();
  }
  if (id != DWT_C0_DEV_ID && id != DWT_C0_PDOA_DEV_ID) {
    char line[40];
    snprintf(line, sizeof(line), "DW RECOVER (id %08lX)", (unsigned long)id);
    say(line);
    n_recover++;
    dw_start();
  }

  uint8_t n_left = 0;
  for (uint8_t i = 0; i < N_ANCHORS; i++) {
    n_left += stats[i].enabled;
  }
  for (uint8_t i = 0; i < N_ANCHORS; i++) {
    if (!stats[i].enabled) {
      continue;
    }
    n_left--; /* anchors still to range after this one */
    stats[i].asked++;
    for (uint8_t t = 0; t < max_tries; t++) {
      /* Retry only if it and one try for every anchor left still fit the frame. */
      if (t > 0 && round_period_ms &&
          (micros() - round_start_us) + (uint32_t)(1 + n_left) * EXCHANGE_MAX_US + FRAME_TAIL_US >
              round_period_ms * 1000UL) {
        n_retry_skip++;
        break;
      }
      bool ok = range_one(i);
      adapt_power(i, ok, last_rx_dbm);
      if (ok) {
        stats[i].ranged++;
        break;
      }
    }
  }
  iir_frame();
  if (round_period_ms && millis() - round_start > round_period_ms) {
    n_overrun++;
  }

  rounds++;
  if (millis() - stats_start_ms >= STATS_PERIOD_MS) {
    print_stats();
  }
  while ((millis() - round_start) < round_period_ms) {
    handle_usb();
  }
}
