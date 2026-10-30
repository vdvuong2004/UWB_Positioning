#include "dw3000.h"
#include <math.h>

#define APP_NAME "CUSTOM BOARD RANGE INIT v1.4"
/*
 * Custom STM32F103RCT6 + DWM3000 board as the SS-TWR initiator, logged over
 * the board's own USB port (USART1 PA9/PA10 gets the same text).
 * Board: Generic STM32F1 series, part "Generic F103RCTx",
 * Tools > USB support: "CDC (generic 'Serial' supersede U(S)ART)".
 * Wiring is in dw3000_board_config.h (SCK PB3, MISO PB4, MOSI PB5, CS PA15,
 * IRQ PC12, RST PD2).
 *
 * Same polls, frames and RX windows as ex_dwm3000_dwm1000_init, so the
 * existing responders answer unchanged:
 *   '3000'  ex_dwm3000_peer_resp on another STM32 + DWM3000 (e.g. the Blue Pill)
 *   'WROOM' ex_dwm3000_wroom_resp on the ESP32-WROOM-32 + DW3000
 *   '1000'  ex_dwm1000_range_resp on the ESP32-C3 + DWM1000
 * PEER_* below set which peers are polled at boot; keys 1/2/3 toggle them.
 * The '1000' poll goes out at full power (0xC0) and blinds a DWM3000 close
 * by, so it is off by default.
 *
 * Output: "DIST <peer> x.xx m" per reply, and every STATS_EVERY rounds the
 * reply count per peer plus mean / std / min / max of the distances. USB commands: p = pause/resume, z = zero the stats,
 * 1 / 2 / 3 = toggle peer 3000 / WROOM / 1000.
 *
 * If the DW3000 stops answering (wrong DEV_ID before a poll), it is reset
 * through RSTn and set up again, and "DW RECOVER" is printed. Some boards
 * hang the chip whenever the receiver runs; this keeps the polls going.
 */

#if !defined(ARDUINO_ARCH_STM32) || !defined(STM32F103xE)
#error "Select Generic STM32F1 series, part Generic F103RCTx."
#endif
#if !defined(USBCON) || !defined(USBD_USE_CDC)
#error "Tools > USB support: CDC (generic 'Serial' supersede U(S)ART)."
#endif

#define PEER_3000 1
#define PEER_WROOM 0
#define PEER_1000 0

const uint8_t PIN_RST = DW3000_PIN_RST;
const uint8_t PIN_IRQ = DW3000_PIN_IRQ;
const uint8_t PIN_SS = DW3000_PIN_CS;

static dwt_config_t config = {
    5,
    DWT_PLEN_128,
    DWT_PAC8,
    9,
    9,
    0, /* IEEE 8-symbol SFD. Required for DWM1000 at 6.8 Mb/s. */
    DWT_BR_6M8,
    DWT_PHRMODE_STD,
    DWT_PHRRATE_STD,
    (129 + 8 - 8),
    DWT_STS_MODE_OFF,
    DWT_STS_LEN_64,
    DWT_PDOA_M0
};

#define RNG_DELAY_MS 500
#define STATS_EVERY 20
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385

static uint8_t tx_poll_1000[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xE0, 0, 0};
static uint8_t rx_resp_1000[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8_t tx_poll_3000[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'D', 'W', '3', 'A', 0xE2, 0, 0};
static uint8_t rx_resp_3000[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'A', '3', 'W', 'D', 0xE3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8_t tx_poll_wroom[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'R', '3', '2', 0xE4, 0, 0};
static uint8_t rx_resp_wroom[] = {0x41, 0x88, 0, 0xCA, 0xDE, '2', '3', 'R', 'W', 0xE5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
#define ALL_MSG_COMMON_LEN 10
#define ALL_MSG_SN_IDX 2
#define RESP_MSG_POLL_RX_TS_IDX 10
#define RESP_MSG_RESP_TX_TS_IDX 14

static uint8_t frame_seq_nb = 0;
#define RX_BUF_LEN 24
static uint8_t rx_buffer[RX_BUF_LEN];
static uint32_t status_reg = 0;

/* DWM1000 replies in about 1.5 ms. */
#define POLL_TX_TO_RESP_RX_DLY_UUS 1000
#define RESP_RX_TIMEOUT_UUS 40000
/* STM32 peer replies at 2.5 ms. The timeout still covers a 30 ms reply. */
#define PEER_RX_DLY_UUS 2000
#define PEER_RX_TIMEOUT_UUS 50000
/* WROOM replies at 2.5 ms. Keep RX open well past that. */
#define WROOM_RX_DLY_UUS 500
#define WROOM_RX_TIMEOUT_UUS 20000

static dwt_txconfig_t txconfig_1000 = { 0x34, 0xC0C0C0C0, 0x0 };
/* Lower than the DWM1000 poll. 0xC0 saturates the other DWM3000, so it never replies. */
static dwt_txconfig_t txconfig_peer = { 0x34, 0x48484848, 0x0 };

struct peer_stats {
  const char *tag;
  uint16_t polls;
  uint16_t replies;
  /* Distance spread over the current STATS_EVERY window, in cm. */
  uint16_t n;
  float sum;
  float sum_sq;
  int min_cm;
  int max_cm;
};
static peer_stats stats_3000 = {"3000"};
static peer_stats stats_wroom = {"WROOM"};
static peer_stats stats_1000 = {"1000"};
static uint16_t rounds = 0;
static uint16_t n_recover = 0;
static bool use_3000 = PEER_3000;
static bool use_wroom = PEER_WROOM;
static bool use_1000 = PEER_1000;
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

static void print_peers(void)
{
  char line[48];
  snprintf(line, sizeof(line), "peers:%s%s%s", use_3000 ? " 3000" : "",
           use_wroom ? " WROOM" : "", use_1000 ? " 1000" : "");
  say(line);
}

static void banner(void)
{
  say(APP_NAME);
  say(dw3000_clock_from_hse ? "CLOCK HSE 72 MHz" : "CLOCK HSI 48 MHz (crystal failed)");
  print_peers();
  say("commands: p = pause/resume, z = zero stats, 1/2/3 = toggle 3000/WROOM/1000");
}

/* spiSelect() pulses RSTn, so this is also the recovery path. */
static bool dw_start(void)
{
  spiSelect(PIN_SS);
  delay(5);

  /* Bit-banged SPI can miss IDLE_RC on the first reads; a valid ID is enough. */
  bool idle = false;
  uint32_t dev_id = 0;
  uint32_t status = 0;
  for (int attempt = 0; attempt < 50; attempt++) {
    dev_id = dwt_readdevid();
    status = dwt_read32bitreg(SYS_STATUS_ID);
    if (dwt_checkidlerc() || dev_id == DWT_C0_DEV_ID || dev_id == DWT_C0_PDOA_DEV_ID) {
      idle = true;
      break;
    }
    delay(2);
  }
  char line[48];
  if (!idle) {
    snprintf(line, sizeof(line), "IDLE FAILED id %08lX st %08lX",
             (unsigned long)dev_id, (unsigned long)status);
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
  dwt_configuretxrf(&txconfig_1000);
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
  wait_usb_host();
  banner();

  spiBegin(PIN_IRQ, PIN_RST);
  if (!dw_start()) {
    halt("DW3000 START FAILED");
  }
  say("ranging");
}

static void range_one(uint8_t *poll, uint8_t *expect, peer_stats *st,
                      dwt_txconfig_t *tx, uint32_t rx_dly, uint32_t rx_to, uint32_t wait_ms)
{
  dwt_configuretxrf(tx);
  dwt_setrxaftertxdelay(rx_dly);
  dwt_setrxtimeout(rx_to);

  poll[ALL_MSG_SN_IDX] = frame_seq_nb;
  /* A leftover RX timeout bit would abort the next poll before it is sent. */
  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
  dwt_writetxdata(sizeof(tx_poll_1000), poll, 0);
  dwt_writetxfctrl(sizeof(tx_poll_1000), 0, 1);
  dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
  st->polls++;

  /* Read SYS_STATUS twice and drop bit-banged SPI glitch reads. */
  uint32_t wait_start = millis();
  bool tx_done = false;
  while (1) {
    uint32_t a = dwt_read32bitreg(SYS_STATUS_ID);
    uint32_t b = dwt_read32bitreg(SYS_STATUS_ID);
    if (a != b || (a & 0xC0000000UL) != 0 || ((a >> 16) & 0xFFFFUL) == 0xCA03UL) {
      if ((millis() - wait_start) > wait_ms) {
        status_reg = a;
        break;
      }
      continue;
    }
    status_reg = a;
    if (status_reg & SYS_STATUS_TXFRS_BIT_MASK) {
      tx_done = true;
    }
    if (status_reg & SYS_STATUS_RXFCG_BIT_MASK) {
      break;
    }
    /* Ignore a timeout that belongs to the previous exchange. CIAERR alone
     * can come before the CRC result, so it does not end the wait. */
    if (tx_done && (status_reg & (SYS_STATUS_ALL_RX_TO | (SYS_STATUS_ALL_RX_ERR & ~SYS_STATUS_CIAERR_BIT_MASK)))) {
      break;
    }
    if ((millis() - wait_start) > wait_ms) {
      break;
    }
  }
  frame_seq_nb++;

  char line[48];
  if (status_reg & SYS_STATUS_RXFCG_BIT_MASK) {
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
    uint32_t frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
    if (frame_len <= sizeof(rx_buffer)) {
      dwt_readrxdata(rx_buffer, frame_len, 0);
      rx_buffer[ALL_MSG_SN_IDX] = 0;
      if (memcmp(rx_buffer, expect, ALL_MSG_COMMON_LEN) == 0) {
        uint32_t poll_tx_ts = dwt_readtxtimestamplo32();
        uint32_t resp_rx_ts = dwt_readrxtimestamplo32();
        int16_t clk_a = dwt_readclockoffset();
        int16_t clk_b = dwt_readclockoffset();
        double clockOffsetRatio = 0.0;
        if (clk_a == clk_b) {
          clockOffsetRatio = (double)clk_a / (double)(1UL << 26);
        }
        uint32_t poll_rx_ts, resp_tx_ts;
        resp_msg_get_ts(&rx_buffer[RESP_MSG_POLL_RX_TS_IDX], &poll_rx_ts);
        resp_msg_get_ts(&rx_buffer[RESP_MSG_RESP_TX_TS_IDX], &resp_tx_ts);

        int32_t rtd_init = (int32_t)(resp_rx_ts - poll_tx_ts);
        int32_t rtd_resp = (int32_t)(resp_tx_ts - poll_rx_ts);
        double tof = (((double)rtd_init - (double)rtd_resp * (1.0 - clockOffsetRatio)) / 2.0) * DWT_TIME_UNITS;
        double distance = tof * SPEED_OF_LIGHT;

        /* newlib-nano printf has no %f. */
        int cm = (int)(distance * 100.0 + (distance < 0 ? -0.5 : 0.5));
        char sign = ' ';
        if (cm < 0) {
          sign = '-';
          cm = -cm;
        }
        st->replies++;
        int scm = sign == '-' ? -cm : cm;
        if (st->n == 0 || scm < st->min_cm) {
          st->min_cm = scm;
        }
        if (st->n == 0 || scm > st->max_cm) {
          st->max_cm = scm;
        }
        st->n++;
        st->sum += scm;
        st->sum_sq += (float)scm * scm;
        snprintf(line, sizeof(line), "DIST %s %c%d.%02d m", st->tag, sign, cm / 100, cm % 100);
        say(line);
        return;
      }
    }
    snprintf(line, sizeof(line), "BAD RESP %s", st->tag);
  } else {
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
    snprintf(line, sizeof(line), "NO RESP %s %08lX", st->tag, (unsigned long)status_reg);
  }
  say(line);
}

static void print_stats(void)
{
  char line[80];
  snprintf(line, sizeof(line), "STATS 3000 %u/%u  WROOM %u/%u  1000 %u/%u  recover %u",
           stats_3000.replies, stats_3000.polls, stats_wroom.replies, stats_wroom.polls,
           stats_1000.replies, stats_1000.polls, n_recover);
  say(line);

  peer_stats *all[] = {&stats_3000, &stats_wroom, &stats_1000};
  for (peer_stats *st : all) {
    if (st->n == 0) {
      continue;
    }
    float mean = st->sum / st->n;
    float var = st->sum_sq / st->n - mean * mean;
    int mean_cm = (int)(mean + (mean < 0 ? -0.5f : 0.5f));
    int std_cm = (int)(sqrtf(var > 0 ? var : 0) + 0.5f);
    /* newlib-nano printf has no %f, so print cm. */
    snprintf(line, sizeof(line), "SPREAD %s n %u  mean %d cm  std %d cm  min %d  max %d",
             st->tag, st->n, mean_cm, std_cm, st->min_cm, st->max_cm);
    say(line);
    st->n = 0;
    st->sum = st->sum_sq = 0;
  }
}

static void zero_stats(void)
{
  stats_3000.polls = stats_3000.replies = 0;
  stats_wroom.polls = stats_wroom.replies = 0;
  stats_1000.polls = stats_1000.replies = 0;
  rounds = 0;
  n_recover = 0;
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
      zero_stats();
      say("stats zeroed");
    } else if (c == '1' || c == '2' || c == '3') {
      bool *flag = (c == '1') ? &use_3000 : (c == '2') ? &use_wroom : &use_1000;
      *flag = !*flag;
      print_peers();
    }
  }
}

void loop()
{
  handle_usb();
  if (paused) {
    return;
  }

  uint32_t id = dwt_readdevid();
  if (id != DWT_C0_DEV_ID && id != DWT_C0_PDOA_DEV_ID) {
    char line[40];
    snprintf(line, sizeof(line), "DW RECOVER (id %08lX)", (unsigned long)id);
    say(line);
    n_recover++;
    dw_start();
  }

  /* STM32 peer first. It is back in RX after the delay below. The DWM1000
   * poll would otherwise occupy the peer while its own poll is sent. */
  if (use_3000) {
    range_one(tx_poll_3000, rx_resp_3000, &stats_3000, &txconfig_peer,
              PEER_RX_DLY_UUS, PEER_RX_TIMEOUT_UUS, 150);
  }
  if (use_wroom) {
    range_one(tx_poll_wroom, rx_resp_wroom, &stats_wroom, &txconfig_peer,
              WROOM_RX_DLY_UUS, WROOM_RX_TIMEOUT_UUS, 40);
  }
  if (use_1000) {
    range_one(tx_poll_1000, rx_resp_1000, &stats_1000, &txconfig_1000,
              POLL_TX_TO_RESP_RX_DLY_UUS, RESP_RX_TIMEOUT_UUS, 60);
  }

  if (++rounds >= STATS_EVERY) {
    print_stats();
    rounds = 0;
  }
  Sleep(RNG_DELAY_MS);
}
