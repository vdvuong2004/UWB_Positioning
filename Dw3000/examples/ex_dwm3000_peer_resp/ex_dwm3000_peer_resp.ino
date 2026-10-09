#include "dw3000.h"

#define APP_NAME "DWM3000 PEER RESP v1.3"
/*
 * STM32F103C6T6 + DWM3000 responder. The logger STM32 runs
 * ex_dwm3000_dwm1000_init and prints DIST 3000.
 * Board part number must be STM32F103C6. Sketch must stay under 32 KB.
 * Pins: SCK PA5, MISO PA6, MOSI PA7, CS PA4, IRQ PB1, RST PB0.
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
#if defined(ARDUINO_ARCH_STM32)
/* Bit-bang SPI on the F103C6 needs this long to arm the delayed TX.
 * 15 ms was too short (reply never left). 30 ms is the delay that answered. */
#define POLL_RX_TO_RESP_TX_DLY_UUS 30000
#else
/* Hardware SPI. 2.5 ms leaves time to arm the delayed TX.
 * 1 ppm of clock error is then about 0.4 m. */
#define POLL_RX_TO_RESP_TX_DLY_UUS 2500
#endif

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

static dwt_txconfig_t txconfig_peer = { 0x34, 0x48484848, 0x0 };

static void halt(const char *msg)
{
  test_run_info((unsigned char *)msg);
  while (1) {
    delay(1000);
  }
}

void setup()
{
  UART_init();
  test_run_info((unsigned char *)APP_NAME);

  spiBegin(PIN_IRQ, PIN_RST);
  spiSelect(PIN_SS);

  bool idle = false;
  for (int attempt = 0; attempt < 20; attempt++) {
    if (dwt_checkidlerc()) {
      idle = true;
      break;
    }
    delay(2);
  }
  if (!idle) {
    halt("IDLE FAILED");
  }
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
#if defined(ARDUINO_ARCH_STM32)
  pinMode(PC13, OUTPUT);
  for (int blink = 0; blink < 6; blink++) {
    digitalWrite(PC13, blink & 1);
    delay(100);
  }
  digitalWrite(PC13, HIGH);
#endif
  test_run_info((unsigned char *)"listening for peer poll");
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
#if defined(ARDUINO_ARCH_STM32)
  digitalWrite(PC13, !digitalRead(PC13));
#endif

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
  test_run_info((unsigned char *)"RESP sent");
}
