#include <SPI.h>
#include <DW1000.h>

/*
 * ESP32-C3 SuperMini + DWM1000 answers ex_dwm3000_dwm1000_init.
 * Distance is printed on the DWM3000 board.
 *   RST GPIO2, IRQ GPIO3, SCK GPIO4, MISO GPIO5, MOSI GPIO6, CS GPIO7
 */
const uint8_t PIN_RST = 2;
const uint8_t PIN_IRQ = 3;
const uint8_t PIN_SCK = 4;
const uint8_t PIN_MISO = 5;
const uint8_t PIN_MOSI = 6;
const uint8_t PIN_SS = 7;

volatile boolean received = false;

static uint8_t tx_resp_msg[20] = {
    0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0xE1,
    0, 0, 0, 0, 0, 0, 0, 0};

static void logLine(const char *text) {
#if defined(ARDUINO_ARCH_ESP32) && !ARDUINO_USB_CDC_ON_BOOT
  printf("%s\r\n", text);
#else
  Serial.println(text);
#endif
}

static void startListen() {
  DW1000.newReceive();
  DW1000.startReceive();
}

void setup() {
#if !defined(ARDUINO_ARCH_ESP32) || ARDUINO_USB_CDC_ON_BOOT
  Serial.begin(115200);
  delay(200);
#endif
  logLine("DWM1000 RESP v1.0");

  DW1000.setSpiPins(PIN_SCK, PIN_MISO, PIN_MOSI);
  DW1000.begin(PIN_IRQ, PIN_RST);
  detachInterrupt(digitalPinToInterrupt(PIN_IRQ));
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, -1);
  DW1000.select(PIN_SS);

  char id[128];
  char line[160];
  DW1000.getPrintableDeviceIdentifier(id);
  snprintf(line, sizeof(line), "DEV ID %s", id);
  logLine(line);

  DW1000.newConfiguration();
  DW1000.setDefaults();
  DW1000.enableMode(DW1000.MODE_SHORTDATA_FAST_ACCURACY);
  DW1000.setChannel(DW1000.CHANNEL_5);
  DW1000.setPreambleCode(DW1000.PREAMBLE_CODE_64MHZ_9);
  DW1000.setAntennaDelay(16436);
  DW1000.commitConfiguration();

  DW1000.attachReceivedHandler(handleReceived);
  startListen();
  logLine("listening");
}

void handleReceived() {
  received = true;
}

static bool isPoll(const byte *data, uint16_t n) {
  if (n < 10) {
    return false;
  }
  return data[0] == 0x41 && data[1] == 0x88 && data[3] == 0xCA && data[4] == 0xDE &&
         data[5] == 'W' && data[6] == 'A' && data[7] == 'V' && data[8] == 'E' &&
         data[9] == 0xE0;
}

static void replyToPoll(const byte *poll, uint16_t n) {
  byte rxTs[5] = {0};
  DW1000.getReceiveTimestamp(rxTs);

  DW1000.newTransmit();
  /* Short reply. SS-TWR error grows with this delay: 1 ppm of clock
   * offset is about 0.15 m per millisecond of reply time. */
  DW1000Time txTime = DW1000.setDelay(DW1000Time(1500, DW1000Time::MICROSECONDS));
  byte txTs[5] = {0};
  txTime.getTimestamp(txTs);

  tx_resp_msg[2] = (n > 2) ? poll[2] : 0;
  memcpy(tx_resp_msg + 10, rxTs, 4);
  memcpy(tx_resp_msg + 14, txTs, 4);
  DW1000.setData(tx_resp_msg, 18);
  DW1000.startTransmit();

  uint32_t t0 = millis();
  while ((millis() - t0) < 30) {
    DW1000.readSystemEventStatusRegister();
    if (DW1000.isTransmitDone()) {
      break;
    }
  }
  logLine("RESP");
  startListen();
}

void loop() {
  if (digitalRead(PIN_IRQ) == HIGH) {
    DW1000.handleInterrupt();
  }
  if (!received) {
    return;
  }
  received = false;

  uint16_t n = DW1000.getDataLength();
  if (n > 16) {
    n = 16;
  }
  byte poll[16];
  DW1000.getData(poll, n);
  if (isPoll(poll, n)) {
    replyToPoll(poll, n);
  } else {
    startListen();
  }
}
