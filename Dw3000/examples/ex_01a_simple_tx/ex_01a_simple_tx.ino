#include "dw3000.h"

#define APP_NAME "SIMPLE TX v1.1"

/*
 * ESP32-C3 SuperMini + DWM3000
 *   DWM3000 RST  -> GPIO2
 *   DWM3000 IRQ  -> GPIO3
 *   DWM3000 SCK  -> GPIO4
 *   DWM3000 MISO -> GPIO5
 *   DWM3000 MOSI -> GPIO6
 *   DWM3000 CS   -> GPIO7
 *   DWM3000 VCC  -> 3V3
 *   DWM3000 GND  -> GND
 *
 * Arduino board: ESP32C3 Dev Module
 * USB CDC On Boot: Enabled
 */
const uint8_t PIN_RST = DW3000_PIN_RST; // GPIO2
const uint8_t PIN_IRQ = DW3000_PIN_IRQ; // GPIO3
const uint8_t PIN_SS = DW3000_PIN_CS;   // GPIO7

/* Default communication configuration. We use default non-STS DW mode. */
static dwt_config_t config = {
    5,               /* Channel number. */
    DWT_PLEN_128,    /* Preamble length. Used in TX only. */
    DWT_PAC8,        /* Preamble acquisition chunk size. Used in RX only. */
    9,               /* TX preamble code. Used in TX only. */
    9,               /* RX preamble code. Used in RX only. */
    0,               /* IEEE 8-symbol SFD. Matches a DWM1000 at 6.8 Mb/s. Do not use the 4z SFD. */
    DWT_BR_6M8,      /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    DWT_PHRRATE_STD, /* PHY header rate. */
    (129 + 8 - 8),   /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    DWT_STS_MODE_OFF,
    DWT_STS_LEN_64, /* STS length, see allowed values in Enum dwt_sts_lengths_e */
    DWT_PDOA_M0     /* PDOA mode off */
};

/* The frame sent in this example is an 802.15.4e standard blink. It is a 12-byte frame composed of the following fields:
 *     - byte 0: frame type (0xC5 for a blink).
 *     - byte 1: sequence number, incremented for each new frame.
 *     - byte 2 -> 9: device ID, see NOTE 1 below.
 */
static uint8_t tx_msg[] = {0xC5, 0, 'D', 'E', 'C', 'A', 'W', 'A', 'V', 'E'};
/* Index to access to sequence number of the blink frame in the tx_msg array. */
#define BLINK_FRAME_SN_IDX 1

#define FRAME_LENGTH (sizeof(tx_msg) + FCS_LEN) // The real length that is going to be transmitted

/* Inter-frame delay period, in milliseconds. */
#define TX_DELAY_MS 500

/* 0xFDFDFDFD is full power and saturates a DWM1000 on the same bench.
 * 0xC0C0C0C0 is enough for a few metres and lets the DW1000 decode the frame. */
static dwt_txconfig_t txconfig_bench = {
    0x34,       /* PG delay, channel 5. */
    0xC0C0C0C0, /* TX power. */
    0x0
};

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

  /* SPI starts at 2 MHz. RSTn is pulsed inside spiSelect(). */
  spiBegin(PIN_IRQ, PIN_RST);
  spiSelect(PIN_SS);

  /* DW3000 moves from INIT_RC to IDLE_RC after RSTn is released. */
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
  test_run_info((unsigned char *)"INIT OK");

  // Enabling LEDs here for debug so that for each TX the D1 LED will flash on DW3000 red eval-shield boards.
  dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

  // Configure DW IC. See NOTE 5 below.
  if (dwt_configure(&config)) // if the dwt_configure returns DWT_ERROR either the PLL or RX calibration has failed the host should reset the device
  {
    halt("CONFIG FAILED");
  }

  /* Configure the TX spectrum parameters (power, PG delay and PG count). */
  dwt_configuretxrf(&txconfig_bench);
  spiSetFastRate();
  test_run_info((unsigned char *)"CONFIG OK");
}

void loop()
{
  /* Write frame data to DW IC and prepare transmission. See NOTE 3 below.*/
  dwt_writetxdata(FRAME_LENGTH - FCS_LEN, tx_msg, 0); /* Zero offset in TX buffer. */

  /* In this example since the length of the transmitted frame does not change,
   * nor the other parameters of the dwt_writetxfctrl function, the
   * dwt_writetxfctrl call could be outside the main while(1) loop.
   */
  dwt_writetxfctrl(FRAME_LENGTH, 0, 0); /* Zero offset in TX buffer, no ranging. */

  /* Start transmission. */
  dwt_starttx(DWT_START_TX_IMMEDIATE);

  /* Poll DW IC until TX frame sent event set. See NOTE 4 below.
   * STATUS register is 4 bytes long but, as the event we are looking at is in the first byte of the register, we can use this simplest API
   * function to access it.*/
  uint32_t tx_start = millis();
  while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK))
  {
    if ((millis() - tx_start) > 100) {
      dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_TX);
      test_run_info((unsigned char *)"TX TIMEOUT");
      return;
    }
  }

  /* Clear TX frame sent event. */
  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);

  /* This example is a beacon: one blink every TX_DELAY_MS, sequence increments forever. */
  char tx_line[40];
  snprintf(tx_line, sizeof(tx_line), "TX seq %u", tx_msg[BLINK_FRAME_SN_IDX]);
  test_run_info((unsigned char *)tx_line);

  Sleep(TX_DELAY_MS);

  /* Increment the blink frame sequence number (modulo 256). */
  tx_msg[BLINK_FRAME_SN_IDX]++;
}

/*****************************************************************************************************************************************************
 * NOTES:
 *
 * 1. The device ID is a hard coded constant in the blink to keep the example simple but for a real product every device should have a unique ID.
 *    For development purposes it is possible to generate a DW IC unique ID by combining the Lot ID & Part Number values programmed into the
 *    DW IC during its manufacture. However there is no guarantee this will not conflict with someone else�s implementation. We recommended that
 *    customers buy a block of addresses from the IEEE Registration Authority for their production items. See "EUI" in the DW IC User Manual.
 * 2. In a real application, for optimum performance within regulatory limits, it may be necessary to set TX pulse bandwidth and TX power, (using
 *    the dwt_configuretxrf API call) to per device calibrated values saved in the target system or the DW IC OTP memory.
 * 3. dwt_writetxdata() takes the full size of tx_msg as a parameter but only copies (size - 2) bytes as the check-sum at the end of the frame is
 *    automatically appended by the DW IC. This means that our tx_msg could be two bytes shorter without losing any data (but the sizeof would not
 *    work anymore then as we would still have to indicate the full length of the frame to dwt_writetxdata()).
 * 4. We use polled mode of operation here to keep the example as simple as possible, but the TXFRS status event can be used to generate an interrupt.
 *    Please refer to DW IC User Manual for more details on "interrupts".
 * 5. Desired configuration by user may be different to the current programmed configuration. dwt_configure is called to set desired
 *    configuration.
 ****************************************************************************************************************************************************/
