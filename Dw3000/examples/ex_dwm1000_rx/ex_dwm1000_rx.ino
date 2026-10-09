#include <SPI.h>
#include <DW1000.h>

/*
 * ESP32-C3 SuperMini + DWM1000 listens for ex_01a_simple_tx on a DWM3000.
 * Same wiring as the DWM3000 on this board:
 *   RST GPIO2, IRQ GPIO3, SCK GPIO4, MISO GPIO5, MOSI GPIO6, CS GPIO7
 * Arduino: ESP32C3 Dev Module. With USB CDC On Boot disabled, the log
 * goes to the USB serial console through printf.
 *
 * Both sides: channel 5, 6.8 Mb/s, 64 MHz PRF, preamble 128, code 9, IEEE SFD.
 */
const uint8_t PIN_RST = 2;
const uint8_t PIN_IRQ = 3;
const uint8_t PIN_SCK = 4;
const uint8_t PIN_MISO = 5;
const uint8_t PIN_MOSI = 6;
const uint8_t PIN_SS = 7;

volatile boolean received = false;
volatile boolean error = false;
volatile uint32_t errorStatus = 0;

static void logLine(const char *text) {
#if defined(ARDUINO_ARCH_ESP32) && !ARDUINO_USB_CDC_ON_BOOT
  printf("%s\r\n", text);
#else
  Serial.println(text);
#endif
}

void setup() {
#if !defined(ARDUINO_ARCH_ESP32) || ARDUINO_USB_CDC_ON_BOOT
  Serial.begin(115200);
  delay(200);
#endif
  logLine("DWM1000 RX v1.0");

  DW1000.setSpiPins(PIN_SCK, PIN_MISO, PIN_MOSI);
  DW1000.begin(PIN_IRQ, PIN_RST);
  /* begin() arms the IRQ. SPI inside that handler deadlocks commitConfiguration
   * and trips the interrupt watchdog. Poll the pin from loop() instead. */
  detachInterrupt(digitalPinToInterrupt(PIN_IRQ));
  /* ss = -1 so the ESP32 driver does not pulse GPIO7 between bytes. */
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, -1);
  DW1000.select(PIN_SS);

  char id[128];
  char line[160];
  DW1000.getPrintableDeviceIdentifier(id);
  snprintf(line, sizeof(line), "DEV ID %s", id);
  logLine(line);
  logLine("cfg");

  DW1000.newConfiguration();
  DW1000.setDefaults();
  DW1000.enableMode(DW1000.MODE_SHORTDATA_FAST_ACCURACY);
  DW1000.setChannel(DW1000.CHANNEL_5);
  DW1000.setPreambleCode(DW1000.PREAMBLE_CODE_64MHZ_9);
  DW1000.setDeviceAddress(6);
  DW1000.setNetworkId(10);
  logLine("commit");
  DW1000.commitConfiguration();

  DW1000.getPrintableDeviceMode(id);
  snprintf(line, sizeof(line), "MODE %s", id);
  logLine(line);

  DW1000.attachReceivedHandler(handleReceived);
  DW1000.attachReceiveFailedHandler(handleError);
  DW1000.attachErrorHandler(handleError);

  DW1000.newReceive();
  DW1000.receivePermanently(true);
  DW1000.startReceive();
  logLine("listening");
}

void handleReceived() {
  received = true;
}

void handleError() {
  errorStatus = DW1000.getLastSystemStatus();
  error = true;
}

void loop() {
  if (digitalRead(PIN_IRQ) == HIGH) {
    DW1000.handleInterrupt();
  }
  if (received) {
    received = false;
    uint16_t n = DW1000.getDataLength();
    if (n > 24) {
      n = 24;
    }
    byte data[24];
    DW1000.getData(data, n);
    char line[160];
    int used = snprintf(line, sizeof(line), "RX %u", n);
    for (uint16_t i = 0; i < n && used > 0 && used < (int)sizeof(line) - 4; i++) {
      used += snprintf(line + used, sizeof(line) - used, " %02X", data[i]);
    }
    logLine(line);
  }
  if (error) {
    uint32_t status = errorStatus;
    error = false;
    if (status == 0xFFFFFF7FUL || status == 0xFFFFFFFFUL) {
      logLine("RX error SPI");
      return;
    }
    char line[160];
    snprintf(line, sizeof(line),
             "RX error sts %08lX%s%s%s%s%s%s",
             (unsigned long)status,
             (status & (1UL << 12)) ? " PHR" : "",
             (status & (1UL << 15)) ? " CRC" : "",
             (status & (1UL << 16)) ? " RS" : "",
             (status & (1UL << 18)) ? " LDE" : "",
             (status & (1UL << 24)) ? " RFPLL" : "",
             (status & (1UL << 25)) ? " CLKPLL" : "");
    logLine(line);
  }
}
