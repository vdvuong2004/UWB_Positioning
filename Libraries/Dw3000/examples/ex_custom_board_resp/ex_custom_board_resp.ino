#include "dw3000.h"

#define APP_NAME "CUSTOM BOARD RANGE RESP v1.6"
/*
 * Custom STM32F103RCT6 + DWM3000 board as the SS-TWR responder '3000',
 * logged over the board's own USB port (USART1 PA9/PA10 gets the same text).
 * Board: Generic STM32F1 series, part "Generic F103RCTx",
 * Tools > USB support: "CDC (generic 'Serial' supersede U(S)ART)".
 *
 * Answers only poll 0xE2 'DW3A' with 0xE3 'A3WD' 3 ms later, like ex_dwm3000_peer_resp,
 * so the other custom board running ex_custom_board_init (or the Blue Pill
 * running ex_dwm3000_dwm1000_init) prints "DIST 3000".
 * Same radio as the other modules: channel 5, PLEN 128, IEEE SFD, 6.8 Mb/s.
 *
 * Output: "RESP sent" per reply when verbose, and every 10 s the number of
 * replies, LATE and RX errors. USB commands: v = verbose on/off, z = zero stats.
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

#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385

/* Poll RX to dwt_starttx measured 1.9 ms worst case with the custom board's
 * fast bit-bang (DW3000_STM32_BB_LOOPS 2); 3 ms leaves 1.5x margin. With the
 * default slow bit-bang this needs 20000. A shorter reply means less SS-TWR
 * clock-drift error. The initiator opens RX ~2.2 ms after its poll. */
#define POLL_RX_TO_RESP_TX_DLY_UUS 3000

#define STATS_PERIOD_MS 10000

static uint8_t rx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'D', 'W', '3', 'A', 0xE2, 0, 0};
/* 18 payload bytes plus 2 CRC placeholders. The chip overwrites the last
 * two with the real CRC, so a 19-byte buffer clips the TX timestamp. */
static uint8_t tx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'A', '3', 'W', 'D', 0xE3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
#define ALL_MSG_COMMON_LEN 10
#define ALL_MSG_SN_IDX 2
#define RESP_MSG_POLL_RX_TS_IDX 10
#define RESP_MSG_RESP_TX_TS_IDX 14

#define RX_BUF_LEN 32
static uint8_t rx_buffer[RX_BUF_LEN];
static uint8_t frame_seq_nb = 0;
static uint32_t status_reg = 0;
static uint64_t poll_rx_ts;
static uint64_t resp_tx_ts;

/* 0xC0C0C0C0 saturates the other DWM3000, which then never decodes the reply. */
static dwt_txconfig_t txconfig_peer = { 0x34, 0x48484848, 0x0 };

static uint32_t n_resp = 0;
static uint32_t n_late = 0;
static uint32_t n_rx_err = 0;
static uint32_t n_other = 0;
/* Per-bit RX error counts, to tell noise (RXSTO/RXPHE) from bad frames (RXFCE). */
static const struct {
  uint32_t mask;
  const char *name;
} rx_err_bits[] = {
  {SYS_STATUS_RXPHE_BIT_MASK, "PHE"},
  {SYS_STATUS_RXFCE_BIT_MASK, "FCE"},
  {SYS_STATUS_RXFSL_BIT_MASK, "FSL"},
  {SYS_STATUS_RXSTO_BIT_MASK, "STO"},
  {SYS_STATUS_ARFE_BIT_MASK, "ARFE"},
  {SYS_STATUS_CIAERR_BIT_MASK, "CIA"},
};
#define N_RX_ERR_BITS (sizeof(rx_err_bits) / sizeof(rx_err_bits[0]))
static uint32_t n_err_bit[N_RX_ERR_BITS];
static uint32_t last_err_status = 0;
static uint32_t n_cia_good = 0;   /* CIAERR on a frame whose CRC was still good */
static uint32_t n_clear_fail = 0; /* status bits still set after the clear write */
static uint32_t max_prep_us = 0;  /* poll RX timestamp to dwt_starttx, worst case */
/* Last frame that was not our poll, to see who else is on the air. */
static uint8_t last_other[12];
static uint32_t last_other_len = 0;
static bool verbose = true;

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

static void banner(void)
{
  say(APP_NAME);
  say(dw3000_clock_from_hse ? "CLOCK HSE 72 MHz" : "CLOCK HSI 48 MHz (crystal failed)");
  say("answers poll 3000 (DW3A 0xE2)");
  say("commands: v = verbose on/off, z = zero stats");
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
    halt("CONFIG FAILED");
  }
  dwt_configuretxrf(&txconfig_peer);
  spiSetFastRate();
  dwt_setrxantennadelay(RX_ANT_DLY);
  dwt_settxantennadelay(TX_ANT_DLY);
  dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);
  say("listening for poll 3000");
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
    if (c == 'v' || c == 'V') {
      verbose = !verbose;
      say(verbose ? "verbose on" : "verbose off");
    } else if (c == 'z' || c == 'Z') {
      n_resp = n_late = n_rx_err = n_other = n_cia_good = n_clear_fail = max_prep_us = 0;
      memset(n_err_bit, 0, sizeof(n_err_bit));
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
  snprintf(line, sizeof(line), "STATS resp %lu  late %lu  rx err %lu  other frames %lu",
           (unsigned long)n_resp, (unsigned long)n_late, (unsigned long)n_rx_err, (unsigned long)n_other);
  say(line);
  int len = snprintf(line, sizeof(line), "  err bits");
  for (uint8_t i = 0; i < N_RX_ERR_BITS && len < (int)sizeof(line); i++) {
    len += snprintf(line + len, sizeof(line) - len, " %s %lu", rx_err_bits[i].name, (unsigned long)n_err_bit[i]);
  }
  say(line);
  snprintf(line, sizeof(line), "  last err status %08lX  cia on good frame %lu  clear failed %lu",
           (unsigned long)last_err_status, (unsigned long)n_cia_good, (unsigned long)n_clear_fail);
  say(line);
  snprintf(line, sizeof(line), "  reply prep max %lu us (reply delay %u us)",
           (unsigned long)max_prep_us, (unsigned)POLL_RX_TO_RESP_TX_DLY_UUS);
  say(line);
  if (last_other_len) {
    len = snprintf(line, sizeof(line), "  last other frame len %lu:", (unsigned long)last_other_len);
    for (uint8_t i = 0; i < sizeof(last_other) && i < last_other_len && len < (int)sizeof(line); i++) {
      len += snprintf(line + len, sizeof(line) - len, " %02X", last_other[i]);
    }
    say(line);
  }
}

void loop()
{
  handle_usb();
  print_stats();

  /* A leftover error bit would end the wait below before any frame arrives. */
  const uint32_t rx_bits = SYS_STATUS_ALL_RX_GOOD | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_TO;
  dwt_write32bitreg(SYS_STATUS_ID, rx_bits);
  if (dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_ALL_RX_ERR) {
    n_clear_fail++;
  }
  dwt_rxenable(DWT_START_RX_IMMEDIATE);

  /* Wake up every 200 ms so USB commands and stats keep running with no poll. */
  uint32_t wait_start = millis();
  while (1) {
    /* Read twice and drop bit-banged SPI glitch reads, as the initiator does.
     * A glitch with an error bit set would restart RX in the middle of a poll. */
    uint32_t a = dwt_read32bitreg(SYS_STATUS_ID);
    uint32_t b = dwt_read32bitreg(SYS_STATUS_ID);
    if (a == b && (a & 0xC0000000UL) == 0 && ((a >> 16) & 0xFFFFUL) != 0xCA03UL) {
      status_reg = a;
      /* CIAERR alone is not fatal: it can come before the CRC result, and a
       * frame with a good CRC is still worth answering. */
      if (status_reg & (SYS_STATUS_RXFCG_BIT_MASK | (SYS_STATUS_ALL_RX_ERR & ~SYS_STATUS_CIAERR_BIT_MASK))) {
        break;
      }
    }
    if ((millis() - wait_start) > 200) {
      dwt_writefastCMD(CMD_TXRXOFF);
      if (status_reg & SYS_STATUS_CIAERR_BIT_MASK) {
        n_rx_err++;
        n_err_bit[N_RX_ERR_BITS - 1]++;
        last_err_status = status_reg;
      }
      return;
    }
  }

  if ((status_reg & SYS_STATUS_RXFCG_BIT_MASK) && (status_reg & SYS_STATUS_CIAERR_BIT_MASK)) {
    n_cia_good++;
  }
  if (!(status_reg & SYS_STATUS_RXFCG_BIT_MASK)) {
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
    n_rx_err++;
    last_err_status = status_reg;
    for (uint8_t i = 0; i < N_RX_ERR_BITS; i++) {
      if (status_reg & rx_err_bits[i].mask) {
        n_err_bit[i]++;
      }
    }
    return;
  }

  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
  uint32_t frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
  if (frame_len > sizeof(rx_buffer)) {
    frame_len = sizeof(rx_buffer);
  }
  if (frame_len == 0) {
    return;
  }

  dwt_readrxdata(rx_buffer, frame_len, 0);
  rx_buffer[ALL_MSG_SN_IDX] = 0;
  if (memcmp(rx_buffer, rx_poll_msg, ALL_MSG_COMMON_LEN) != 0) {
    /* Polls for the WROOM or the DWM1000 land here. */
    n_other++;
    last_other_len = frame_len;
    memcpy(last_other, rx_buffer, sizeof(last_other));
    return;
  }

  poll_rx_ts = get_rx_timestamp_u64();
  uint32_t resp_tx_time = (uint32_t)((poll_rx_ts + ((uint64_t)POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8);
  dwt_setdelayedtrxtime(resp_tx_time);
  resp_tx_ts = (((uint64_t)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;
  resp_msg_set_ts(&tx_resp_msg[RESP_MSG_POLL_RX_TS_IDX], poll_rx_ts);
  resp_msg_set_ts(&tx_resp_msg[RESP_MSG_RESP_TX_TS_IDX], resp_tx_ts);
  tx_resp_msg[ALL_MSG_SN_IDX] = frame_seq_nb;

  dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0);
  dwt_writetxfctrl(sizeof(tx_resp_msg), 0, 1);
  /* A stale TXFRS from the last reply would end the wait below at once. */
  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);

  /* Time from the poll RX timestamp to here, on the DW3000 clock (hi32 is
   * in 256 x 15.65 ps units), so the reply delay can be set from data. */
  uint32_t prep = dwt_readsystimestamphi32() - (uint32_t)(poll_rx_ts >> 8);
  uint32_t prep_us = (uint32_t)(((uint64_t)prep * 4006u) / 1000000u);
  if (prep_us > max_prep_us) {
    max_prep_us = prep_us;
  }

  /* Log only after the reply is scheduled: printing first would make it LATE. */
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
  frame_seq_nb++;
  n_resp++;
  if (verbose) {
    say("RESP sent");
  }
}
