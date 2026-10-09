#include "dw3000.h"

/*
 * ESP32-C3 SuperMini + DWM3000
 *   RST GPIO2, IRQ GPIO3, SCK GPIO4, MISO GPIO5, MOSI GPIO6, CS GPIO7
 *
 * Prints the DW3000 device ID once a second so the USB serial monitor
 * still shows it after the port connects. Expected ID is DECA0302 or DECA0312.
 */

const uint8_t PIN_RST = DW3000_PIN_RST;
const uint8_t PIN_IRQ = DW3000_PIN_IRQ;
const uint8_t PIN_SS  = DW3000_PIN_CS;

static void reportDevId(void)
{
  uint32_t dev_id = dwt_readdevid();
  char line[48];

  snprintf(line, sizeof(line), "DEVICE ID: %08lX", (unsigned long)dev_id);
  test_run_info((unsigned char *)line);

  if ((dev_id == DWT_C0_DEV_ID) || (dev_id == DWT_C0_PDOA_DEV_ID)) {
    test_run_info((unsigned char *)"DEV ID OK");
  } else {
    test_run_info((unsigned char *)"DEV ID FAILED");
  }
}

void setup()
{
  UART_init();
  delay(300);
  test_run_info((unsigned char *)"READ DEV ID");

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
    test_run_info((unsigned char *)"IDLE FAILED");
  }

  reportDevId();
}

void loop()
{
  reportDevId();
  delay(1000);
}
