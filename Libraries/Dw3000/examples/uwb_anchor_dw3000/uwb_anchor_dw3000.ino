#include "dw3000.h"

#define APP_NAME "UWB ANCHOR DW3000 v2.0"
/*
 * UWB anchor (SS-TWR responder) with a DWM3000. Answers polls addressed to
 * ANCHOR_ID from uwb_tag. Give every anchor its own ID and list it in the
 * tag's ANCHORS[] table.
 *
 * Boards (pins in src/dw3000_board_config.h):
 *   Custom STM32F103RCT6 board: "Generic STM32F1 series", part "Generic F103RCTx",
 *     Tools > USB support: "CDC (generic 'Serial' supersede U(S)ART)". Log on USB.
 *   ESP32-WROOM-32 (test station): "ESP32 Dev Module". Log on Serial 115200.
 *
 * Commands (USB / Serial): v = verbose on/off, z = zero stats.
 * Every 10 s: STATS with replies, late replies, RX errors and the worst reply
 * prep time, which must stay below REPLY_DELAY_US.
 *
 * Protocol: see uwb_tag. Change the tag and both anchor sketches together.
 */

/* Unique per anchor. Can also be set with -DANCHOR_ID=n. */
#ifndef ANCHOR_ID
#define ANCHOR_ID 1
#endif

#if defined(ARDUINO_ARCH_STM32)
#if !defined(STM32F103xE)
#error "Select Generic STM32F1 series, part Generic F103RCTx."
#endif
#if !defined(USBCON) || !defined(USBD_USE_CDC)
#error "Tools > USB support: CDC (generic 'Serial' supersede U(S)ART)."
#endif
#define CONSOLE SerialUSB
/* 0xC0 saturates a DWM3000 tag that is close by. */
#define TX_POWER 0x48
#elif defined(ARDUINO_ARCH_ESP32)
#define CONSOLE Serial
/* The WROOM test board's reply was not heard by the tag at 0x48. */
#define TX_POWER 0x80
#else
#error "Supported boards: custom STM32F103RCT6 board, ESP32."
#endif

#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385

/* Poll RX to dwt_starttx takes 1.9 ms worst case on the custom board's
 * bit-banged SPI (DW3000_STM32_BB_LOOPS 2). The tag expects this exact delay. */
#define REPLY_DELAY_US 3000

#define STATS_PERIOD_MS 10000

static dwt_config_t config = {
    5, DWT_PLEN_128, DWT_PAC8, 9, 9,
    0, /* IEEE 8-symbol SFD: required for the DWM1000 at 6.8 Mb/s. */
    DWT_BR_6M8, DWT_PHRMODE_STD, DWT_PHRRATE_STD,
    (129 + 8 - 8), DWT_STS_MODE_OFF, DWT_STS_LEN_64, DWT_PDOA_M0
};
static dwt_txconfig_t txconfig = {0x34, TX_POWER * 0x01010101UL, 0x0};

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
#define POLL_LEN 12
/* 18 payload bytes plus 2 CRC placeholders: a shorter buffer clips the TX timestamp. */
#define RESP_LEN 20

static uint8_t resp_msg[RESP_LEN] = {FC_LO, FC_HI, 0, PAN_LO, PAN_HI, 0, 0,
                                     ANCHOR_ID & 0xFF, ANCHOR_ID >> 8, FUNC_RESP};
static uint8_t rx_buffer[32];

static uint32_t n_resp = 0;
static uint32_t n_late = 0;
static uint32_t n_rx_err = 0;
static uint32_t n_other = 0;   /* good frames that are not a poll for this anchor */
static uint32_t max_prep_us = 0;
static uint32_t last_err_status = 0;
static bool verbose = false;

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

static void banner(void)
{
  char line[40];
  say(APP_NAME);
#if defined(ARDUINO_ARCH_STM32)
  say(dw3000_clock_from_hse ? "CLOCK HSE 72 MHz" : "CLOCK HSI 48 MHz (crystal failed)");
#endif
  snprintf(line, sizeof(line), "anchor id %u", ANCHOR_ID);
  say(line);
  say("commands: v = verbose on/off, z = zero stats");
}

void setup()
{
  UART_init();
#if defined(ARDUINO_ARCH_STM32)
  /* Logs printed before the host opens the port are lost, so wait for DTR. */
  uint32_t start = millis();
  while (!SerialUSB && (millis() - start < 5000)) {
    delay(10);
  }
  delay(100);
#endif
  banner();

  spiBegin(DW3000_PIN_IRQ, DW3000_PIN_RST);
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
    halt(line);
  }
  snprintf(line, sizeof(line), "DEVICE ID %08lX", (unsigned long)dev_id);
  say(line);
  if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
    halt("INIT FAILED");
  }
  if (dwt_configure(&config)) {
    halt("CONFIG FAILED (RX cal): check the module supply");
  }
  dwt_configuretxrf(&txconfig);
  spiSetFastRate();
  dwt_setrxantennadelay(RX_ANT_DLY);
  dwt_settxantennadelay(TX_ANT_DLY);
  dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);
  say("listening");
}

static void handle_console(void)
{
#if defined(ARDUINO_ARCH_STM32)
  /* Upload resets the board and drops the COM port. Show the banner again
   * whenever a terminal (re)opens it. */
  static bool host_open = SerialUSB.dtr();
  bool dtr = SerialUSB.dtr();
  if (dtr && !host_open) {
    delay(100);
    banner();
  }
  host_open = dtr;
#endif
  while (CONSOLE.available()) {
    char c = CONSOLE.read();
    if (c == 'v' || c == 'V') {
      verbose = !verbose;
      say(verbose ? "verbose on" : "verbose off");
    } else if (c == 'z' || c == 'Z') {
      n_resp = n_late = n_rx_err = n_other = max_prep_us = 0;
      say("stats zeroed");
    }
  }
}

static void print_stats(void)
{
  static uint32_t last = 0;
  if (millis() - last < STATS_PERIOD_MS) {
    return;
  }
  last = millis();
  char line[96];
  snprintf(line, sizeof(line), "STATS resp %lu  late %lu  rx err %lu  other %lu",
           (unsigned long)n_resp, (unsigned long)n_late, (unsigned long)n_rx_err, (unsigned long)n_other);
  say(line);
  snprintf(line, sizeof(line), "  reply prep max %lu us (reply delay %u us)  last err %08lX",
           (unsigned long)max_prep_us, (unsigned)REPLY_DELAY_US, (unsigned long)last_err_status);
  say(line);
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

static bool is_poll_for_me(const uint8_t *f, uint32_t len)
{
  return len == POLL_LEN && f[0] == FC_LO && f[1] == FC_HI && f[3] == PAN_LO && f[4] == PAN_HI &&
         f[IDX_DST] == (ANCHOR_ID & 0xFF) && f[IDX_DST + 1] == (ANCHOR_ID >> 8) && f[IDX_FUNC] == FUNC_POLL;
}

void loop()
{
  handle_console();
  print_stats();

  /* A leftover error bit would end the wait below before any frame arrives. */
  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_GOOD | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_TO);
  dwt_rxenable(DWT_START_RX_IMMEDIATE);

  /* Wake up every 200 ms so commands and stats keep running with no poll. */
  uint32_t status = 0;
  uint32_t wait_start = millis();
  while (1) {
    /* CIAERR alone is not fatal: it can come before the CRC result. */
    if (read_status(&status) &&
        (status & (SYS_STATUS_RXFCG_BIT_MASK | (SYS_STATUS_ALL_RX_ERR & ~SYS_STATUS_CIAERR_BIT_MASK)))) {
      break;
    }
    if ((millis() - wait_start) > 200) {
      dwt_writefastCMD(CMD_TXRXOFF);
      return;
    }
  }

  if (!(status & SYS_STATUS_RXFCG_BIT_MASK)) {
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
    n_rx_err++;
    last_err_status = status;
    return;
  }
  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
  uint32_t frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
  if (frame_len == 0 || frame_len > sizeof(rx_buffer)) {
    n_other++;
    return;
  }
  dwt_readrxdata(rx_buffer, frame_len, 0);
  if (!is_poll_for_me(rx_buffer, frame_len)) {
    /* Polls for the other anchors land here. */
    n_other++;
    return;
  }

  uint64_t poll_rx_ts = get_rx_timestamp_u64();
  uint32_t resp_tx_time = (uint32_t)((poll_rx_ts + ((uint64_t)REPLY_DELAY_US * UUS_TO_DWT_TIME)) >> 8);
  dwt_setdelayedtrxtime(resp_tx_time);
  uint64_t resp_tx_ts = (((uint64_t)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

  resp_msg[IDX_SEQ] = rx_buffer[IDX_SEQ];
  resp_msg[IDX_DST] = rx_buffer[IDX_SRC];
  resp_msg[IDX_DST + 1] = rx_buffer[IDX_SRC + 1];
  resp_msg_set_ts(&resp_msg[IDX_POLL_RX_TS], poll_rx_ts);
  resp_msg_set_ts(&resp_msg[IDX_RESP_TX_TS], resp_tx_ts);
  dwt_writetxdata(RESP_LEN, resp_msg, 0);
  dwt_writetxfctrl(RESP_LEN, 0, 1);
  /* A stale TXFRS from the last reply would end the wait below at once. */
  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);

  /* Poll RX timestamp to here, on the DW3000 clock (hi32 counts 256 x 15.65 ps). */
  uint32_t prep = dwt_readsystimestamphi32() - (uint32_t)(poll_rx_ts >> 8);
  uint32_t prep_us = (uint32_t)(((uint64_t)prep * 4006u) / 1000000u);
  if (prep_us > max_prep_us) {
    max_prep_us = prep_us;
  }

  /* Log only after the reply is scheduled: printing first would make it late. */
  if (dwt_starttx(DWT_START_TX_DELAYED) != DWT_SUCCESS) {
    n_late++;
    say("LATE");
    return;
  }
  uint32_t tx_wait = millis();
  while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK)) {
    if ((millis() - tx_wait) > 50) {
      say("TX stuck");
      dwt_writefastCMD(CMD_TXRXOFF);
      return;
    }
  }
  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
  n_resp++;
  if (verbose) {
    say("RESP sent");
  }
}
