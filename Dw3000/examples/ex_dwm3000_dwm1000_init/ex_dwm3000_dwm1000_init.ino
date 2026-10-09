#include "dw3000.h"

#define APP_NAME "DWM3000 RANGE INIT v1.5"
/*
 * STM32 + DWM3000. Prints three distances on USART1 (PA9/PA10).
 *   DWM1000 on the ESP32-C3 runs ex_dwm1000_range_resp.
 *   STM32F103C6T6 + DWM3000 runs ex_dwm3000_peer_resp (30 ms reply).
 *   ESP32-WROOM-32 + DW3000 runs ex_dwm3000_wroom_resp (2.5 ms reply).
 * SFD is IEEE so the DWM1000 can hear the poll.
 */

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
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385

static uint8_t tx_poll_1000[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xE0, 0, 0};
static uint8_t rx_resp_1000[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
/* Different poll, so the DWM1000 does not answer at the same time. */
static uint8_t tx_poll_3000[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'D', 'W', '3', 'A', 0xE2, 0, 0};
static uint8_t rx_resp_3000[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'A', '3', 'W', 'D', 0xE3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
/* Third poll. The C6 and the DWM1000 ignore it. */
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
/* F103C6 peer replies at 30 ms. RX stays open past that. */
#define PEER_RX_DLY_UUS 2000
#define PEER_RX_TIMEOUT_UUS 50000
/* WROOM-32 hardware SPI replies at 2.5 ms. */
#define WROOM_RX_DLY_UUS 500
#define WROOM_RX_TIMEOUT_UUS 6000

static dwt_txconfig_t txconfig_1000 = { 0x34, 0xC0C0C0C0, 0x0 };
/* Lower than the DWM1000 poll. 0xC0 saturates the other DWM3000, so it never replies. */
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
#if defined(ARDUINO_ARCH_STM32)
  /* PC13 is active-low. Three flashes before the UART banner: the MCU is running. */
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
  test_run_info((unsigned char *)"waiting for DWM1000, C6 and WROOM");
}

static void range_one(uint8_t *poll, uint8_t *expect, const char *tag,
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
    /* Ignore a timeout that belongs to the previous exchange. */
    if (tx_done && (status_reg & (SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR))) {
      break;
    }
    if ((millis() - wait_start) > wait_ms) {
      break;
    }
  }
  frame_seq_nb++;

  char line[40];
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

        int cm = (int)(distance * 100.0 + (distance < 0 ? -0.5 : 0.5));
        char sign = ' ';
        if (cm < 0) {
          sign = '-';
          cm = -cm;
        }
        snprintf(line, sizeof(line), "DIST %s %c%d.%02d m", tag, sign, cm / 100, cm % 100);
        test_run_info((unsigned char *)line);
        return;
      }
    }
    snprintf(line, sizeof(line), "BAD RESP %s", tag);
  } else {
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
    snprintf(line, sizeof(line), "NO RESP %s %08lX", tag, (unsigned long)status_reg);
  }
  test_run_info((unsigned char *)line);
}

void loop()
{
  /* Peer first. It is back in RX after the delay below. The DWM1000 poll
   * would otherwise occupy the peer while its own poll is sent. */
  range_one(tx_poll_3000, rx_resp_3000, "3000", &txconfig_peer,
            PEER_RX_DLY_UUS, PEER_RX_TIMEOUT_UUS, 150);
  range_one(tx_poll_wroom, rx_resp_wroom, "WROOM", &txconfig_peer,
            WROOM_RX_DLY_UUS, WROOM_RX_TIMEOUT_UUS, 30);
  range_one(tx_poll_1000, rx_resp_1000, "1000", &txconfig_1000,
            POLL_TX_TO_RESP_RX_DLY_UUS, RESP_RX_TIMEOUT_UUS, 60);
  Sleep(RNG_DELAY_MS);
}
