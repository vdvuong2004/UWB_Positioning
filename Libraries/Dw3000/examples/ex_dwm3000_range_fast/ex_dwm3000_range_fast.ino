#include "dw3000.h"

#define APP_NAME "DWM3000 RANGE FAST v1.0"
/*
 * Speed test for the logger STM32 only. Same radio and same polls as
 * ex_dwm3000_dwm1000_init. The other three boards stay on their current sketches.
 * No 500 ms pause. Once a second it prints how many ranges succeeded.
 */

const uint8_t PIN_RST = DW3000_PIN_RST;
const uint8_t PIN_IRQ = DW3000_PIN_IRQ;
const uint8_t PIN_SS = DW3000_PIN_CS;

static dwt_config_t config = {
    5, DWT_PLEN_128, DWT_PAC8, 9, 9,
    0, DWT_BR_6M8, DWT_PHRMODE_STD, DWT_PHRRATE_STD,
    (129 + 8 - 8), DWT_STS_MODE_OFF, DWT_STS_LEN_64, DWT_PDOA_M0
};

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

#define POLL_TX_TO_RESP_RX_DLY_UUS 1000
#define RESP_RX_TIMEOUT_UUS 8000
#define PEER_RX_DLY_UUS 2000
#define PEER_RX_TIMEOUT_UUS 40000
#define WROOM_RX_DLY_UUS 500
#define WROOM_RX_TIMEOUT_UUS 8000

static dwt_txconfig_t txconfig_1000 = { 0x34, 0xC0C0C0C0, 0x0 };
static dwt_txconfig_t txconfig_peer = { 0x34, 0x48484848, 0x0 };

static uint16_t ok_3000 = 0;
static uint16_t ok_wroom = 0;
static uint16_t ok_1000 = 0;
static uint16_t try_3000 = 0;
static uint16_t try_wroom = 0;
static uint16_t try_1000 = 0;
static uint32_t rate_t0 = 0;

static void halt(const char *msg)
{
  test_run_info((unsigned char *)msg);
  while (1) {
    delay(1000);
  }
}

void setup()
{
#if defined(ARDUINO_ARCH_STM32)
  pinMode(PC13, OUTPUT);
  for (int blink = 0; blink < 6; blink++) {
    digitalWrite(PC13, blink & 1);
    delay(80);
  }
  digitalWrite(PC13, HIGH);
#endif
  UART_init();
  test_run_info((unsigned char *)APP_NAME);

  spiBegin(PIN_IRQ, PIN_RST);
  spiSelect(PIN_SS);
  delay(5);

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
  if (!idle) {
    char line[48];
    snprintf(line, sizeof(line), "IDLE FAILED id %08lX st %08lX",
             (unsigned long)dev_id, (unsigned long)status);
    halt(line);
  }
  if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
    halt("INIT FAILED");
  }
  if (dwt_configure(&config)) {
    halt("CONFIG FAILED");
  }
  dwt_configuretxrf(&txconfig_1000);
  spiSetFastRate();
  dwt_setrxantennadelay(RX_ANT_DLY);
  dwt_settxantennadelay(TX_ANT_DLY);
  dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
  dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
  dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);
  rate_t0 = millis();
  test_run_info((unsigned char *)"HZ is successful ranges per second");
}

static bool range_one(uint8_t *poll, uint8_t *expect,
                      dwt_txconfig_t *tx, uint32_t rx_dly, uint32_t rx_to, uint32_t wait_ms)
{
  dwt_configuretxrf(tx);
  dwt_setrxaftertxdelay(rx_dly);
  dwt_setrxtimeout(rx_to);

  poll[ALL_MSG_SN_IDX] = frame_seq_nb;
  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
  dwt_writetxdata(sizeof(tx_poll_1000), poll, 0);
  dwt_writetxfctrl(sizeof(tx_poll_1000), 0, 1);
  dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

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
    if (tx_done && (status_reg & (SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR))) {
      break;
    }
    if ((millis() - wait_start) > wait_ms) {
      break;
    }
  }
  frame_seq_nb++;

  if (!(status_reg & SYS_STATUS_RXFCG_BIT_MASK)) {
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
    return false;
  }

  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
  uint32_t frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
  if (frame_len > sizeof(rx_buffer)) {
    return false;
  }
  dwt_readrxdata(rx_buffer, frame_len, 0);
  rx_buffer[ALL_MSG_SN_IDX] = 0;
  if (memcmp(rx_buffer, expect, ALL_MSG_COMMON_LEN) != 0) {
    return false;
  }

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
  (void)distance;
  return true;
}

static void report(void)
{
  if ((millis() - rate_t0) < 1000) {
    return;
  }
  char line[64];
  snprintf(line, sizeof(line), "HZ 3000 %u/%u  WROOM %u/%u  1000 %u/%u",
           ok_3000, try_3000, ok_wroom, try_wroom, ok_1000, try_1000);
  test_run_info((unsigned char *)line);
  ok_3000 = ok_wroom = ok_1000 = 0;
  try_3000 = try_wroom = try_1000 = 0;
  rate_t0 = millis();
}

void loop()
{
  try_3000++;
  if (range_one(tx_poll_3000, rx_resp_3000, &txconfig_peer,
                PEER_RX_DLY_UUS, PEER_RX_TIMEOUT_UUS, 45)) {
    ok_3000++;
  }
  try_wroom++;
  if (range_one(tx_poll_wroom, rx_resp_wroom, &txconfig_peer,
                WROOM_RX_DLY_UUS, WROOM_RX_TIMEOUT_UUS, 12)) {
    ok_wroom++;
  }
  try_1000++;
  if (range_one(tx_poll_1000, rx_resp_1000, &txconfig_1000,
                POLL_TX_TO_RESP_RX_DLY_UUS, RESP_RX_TIMEOUT_UUS, 12)) {
    ok_1000++;
  }
  report();
}
