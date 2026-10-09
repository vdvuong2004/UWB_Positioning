#include "dw3000.h"

#define APP_NAME "DWM3000 WROOM RESP v1.2"
/*
 * ESP32-WROOM-32 + DW3000 responder. The STM32 logger runs
 * ex_dwm3000_dwm1000_init and prints DIST WROOM.
 * Answers only poll 0xE4. IEEE SFD, same radio as the other modules.
 * Board: ESP32 Dev Module. Serial monitor 115200.
 *   RST GPIO4, IRQ GPIO22, CS GPIO21
 *   SCK GPIO18, MISO GPIO19, MOSI GPIO23
 *   3.3 V and GND from the WROOM 3V3 pin.
 */

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
/* Hardware SPI. Logger opens RX at 500 us, so 2.5 ms is inside that window. */
#define POLL_RX_TO_RESP_TX_DLY_UUS 2500

static uint8_t rx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'R', '3', '2', 0xE4, 0, 0};
/* 18 payload bytes plus 2 CRC placeholders. A shorter buffer clips the TX timestamp. */
static uint8_t tx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, '2', '3', 'R', 'W', 0xE5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
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

static dwt_txconfig_t txconfig_wroom = { 0x34, 0x48484848, 0x0 };

/* Onboard LED of most ESP32 DevKit boards. */
#define LED_PIN 2

static void halt(const char *msg)
{
  test_run_info((unsigned char *)msg);
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(150);
  }
}

void setup()
{
  pinMode(LED_PIN, OUTPUT);
  for (int blink = 0; blink < 6; blink++) {
    digitalWrite(LED_PIN, blink & 1);
    delay(80);
  }
  UART_init();
  test_run_info((unsigned char *)APP_NAME);

  spiBegin(PIN_IRQ, PIN_RST);
  spiSelect(PIN_SS);
  delay(5);

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
    char line[40];
    snprintf(line, sizeof(line), "IDLE FAILED id %08lX", (unsigned long)dev_id);
    halt(line);
  }
  char id_line[32];
  snprintf(id_line, sizeof(id_line), "DEVICE ID %08lX", (unsigned long)dev_id);
  test_run_info((unsigned char *)id_line);
  if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
    halt("INIT FAILED");
  }
  if (dwt_configure(&config)) {
    halt("CONFIG FAILED");
  }
  dwt_configuretxrf(&txconfig_wroom);
  spiSetFastRate();
  dwt_setrxantennadelay(RX_ANT_DLY);
  dwt_settxantennadelay(TX_ANT_DLY);
  dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);
  test_run_info((unsigned char *)"listening for WROOM poll");
}

void loop()
{
  dwt_rxenable(DWT_START_RX_IMMEDIATE);

  uint32_t wait_start = millis();
  while (1) {
    uint32_t a = dwt_read32bitreg(SYS_STATUS_ID);
    if ((a & 0xC0000000UL) == 0) {
      status_reg = a;
      if (status_reg & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR)) {
        break;
      }
    }
    if ((millis() - wait_start) > 2000) {
      dwt_writefastCMD(CMD_TXRXOFF);
      return;
    }
  }

  if (!(status_reg & SYS_STATUS_RXFCG_BIT_MASK)) {
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
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
  int ret = dwt_starttx(DWT_START_TX_DELAYED);
  if (ret != DWT_SUCCESS) {
    test_run_info((unsigned char *)"LATE");
    return;
  }

  uint32_t tx_wait = millis();
  while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK)) {
    if ((millis() - tx_wait) > 50) {
      test_run_info((unsigned char *)"TX stuck");
      dwt_writefastCMD(CMD_TXRXOFF);
      return;
    }
  }
  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
  frame_seq_nb++;
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  test_run_info((unsigned char *)"RESP sent");
}
