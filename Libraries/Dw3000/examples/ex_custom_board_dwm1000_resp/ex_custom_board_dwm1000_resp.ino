#include <SPI.h>
#include <DW1000.h>

#define APP_NAME "CUSTOM BOARD DWM1000 RESP v1.0"

/*
 * Custom STM32F103RCT6 board with a DWM1000 on the DWM3000 header (J7/J8).
 * Answers poll 0xE0 'WAVE' with 0xE1 'VEWA' like ex_dwm1000_range_resp, so
 * ex_custom_board_init ranges to it with peer '1000' (key 3).
 *   SCK PB3, MISO PB4, MOSI PB5, CS PA15, IRQ PC12, RST PD2
 * Uses the DW1000 library (arduino-dw1000), bit-banged on STM32.
 * Tools > USB support: CDC (generic 'Serial' supersede U(S)ART). Log on USB.
 *
 * Boot runs a board check first (DEV_ID x1000, temp/vbat), then listens.
 * Every 10 s: STATS with replies, late replies, RX errors, other frames,
 * the worst reply prep time and a DEV_ID re-read (SPI health).
 * USB commands: v = verbose on/off, z = zero stats, r = rerun board check.
 */
#if !defined(USBCON) || !defined(USBD_USE_CDC)
#error "Tools > USB support: CDC (generic 'Serial' supersede U(S)ART)"
#endif

const uint8_t PIN_RST = PD2;
const uint8_t PIN_IRQ = PC12;
const uint8_t PIN_SCK = PB3;
const uint8_t PIN_MISO = PB4;
const uint8_t PIN_MOSI = PB5;
const uint8_t PIN_SS = PA15;

/* The initiator listens from 1 ms to 41 ms after its poll. Prep time is
 * measured and printed; keep this a little above it. SS-TWR error grows with
 * the delay: 1 ppm of clock offset is about 0.15 m per ms. */
#define REPLY_DLY_US 3000
#define STATS_EVERY_MS 10000

static uint8_t tx_resp_msg[20] = {
    0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0xE1,
    0, 0, 0, 0, 0, 0, 0, 0};

volatile boolean received = false;
volatile boolean rx_failed = false;

static bool verbose = false;
static uint32_t n_resp = 0;
static uint32_t n_late = 0;
static uint32_t n_rx_err = 0;
static uint32_t n_other = 0;
static uint32_t n_tx_stuck = 0;
static uint32_t max_prep_us = 0;
static float last_rx_dbm = 0;
static float last_fp_dbm = 0;
static byte last_other[12];
static uint16_t last_other_len = 0;

static void say(const char *text) {
  if (Serial) {
    Serial.println(text);
  }
}

static uint32_t read_dev_id() {
  byte id[4];
  DW1000.readBytes(DEV_ID, NO_SUB, id, 4);
  return ((uint32_t)id[3] << 24) | ((uint32_t)id[2] << 16) | ((uint32_t)id[1] << 8) | id[0];
}

/* Logs printed before the host opens the port are lost, so wait for DTR. */
static void wait_usb_host() {
  uint32_t start = millis();
  while (!Serial && (millis() - start < 5000)) {
    delay(10);
  }
  delay(100);
}

static void startListen() {
  DW1000.newReceive();
  DW1000.startReceive();
}

static void handleReceived() {
  received = true;
}

static void handleReceiveFailed() {
  rx_failed = true;
}

static bool board_check() {
  char line[96];
  uint32_t id = read_dev_id();
  snprintf(line, sizeof(line), "DEV_ID %08lX (DW1000 is DECA0130)", (unsigned long)id);
  say(line);
  if (id != 0xDECA0130UL) {
    say("[FAIL] DEV_ID: check the module, the header contact and the wiring");
    return false;
  }
  uint32_t t0 = millis();
  uint16_t bad = 0;
  for (uint16_t i = 0; i < 1000; i++) {
    if (read_dev_id() != 0xDECA0130UL) {
      bad++;
    }
  }
  snprintf(line, sizeof(line), "SPI 1000 DEV_ID reads in %lu ms, %u bad",
           (unsigned long)(millis() - t0), bad);
  say(line);
  float temp = 0, vbat = 0;
  DW1000.getTempAndVbat(temp, vbat);
  snprintf(line, sizeof(line), "TEMP %d.%d C  VBAT %d.%02d V", (int)temp, (int)(temp * 10) % 10,
           (int)vbat, (int)(vbat * 100) % 100);
  say(line);
  say(bad == 0 ? "[PASS] SPI" : "[FAIL] SPI: reads differ, suspect contact or power");
  return bad == 0;
}

static void configure() {
  DW1000.newConfiguration();
  DW1000.setDefaults();
  DW1000.enableMode(DW1000.MODE_SHORTDATA_FAST_ACCURACY);
  DW1000.setChannel(DW1000.CHANNEL_5);
  DW1000.setPreambleCode(DW1000.PREAMBLE_CODE_64MHZ_9);
  DW1000.setAntennaDelay(16436);
  DW1000.commitConfiguration();
}

static void banner() {
  say(APP_NAME);
  say("answers poll 1000 (WAVE 0xE0), init key 3 enables it");
  char line[64];
  snprintf(line, sizeof(line), "reply delay %u us", (unsigned)REPLY_DLY_US);
  say(line);
  say("commands: v = verbose on/off, z = zero stats, r = rerun board check");
}

void setup() {
  Serial.begin(115200);
  wait_usb_host();
  banner();

  DW1000.setSpiPins(PIN_SCK, PIN_MISO, PIN_MOSI);
  DW1000.begin(PIN_IRQ, PIN_RST);
  /* Poll the IRQ pin from loop(); SPI inside the ISR would race loop(). */
  detachInterrupt(digitalPinToInterrupt(PIN_IRQ));
  DW1000.select(PIN_SS);

  char id[128];
  char line[160];
  DW1000.getPrintableDeviceIdentifier(id);
  snprintf(line, sizeof(line), "DEV ID %s", id);
  say(line);
  board_check();

  configure();
  DW1000.attachReceivedHandler(handleReceived);
  DW1000.attachReceiveFailedHandler(handleReceiveFailed);
  startListen();
  say("listening");
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
  DW1000Time rx_time;
  DW1000.getReceiveTimestamp(rx_time);
  byte rxTs[5] = {0};
  rx_time.getTimestamp(rxTs);
  last_rx_dbm = DW1000.getReceivePower();
  last_fp_dbm = DW1000.getFirstPathPower();

  DW1000.newTransmit();
  DW1000Time txTime = DW1000.setDelay(DW1000Time((int32_t)REPLY_DLY_US, DW1000Time::MICROSECONDS));
  byte txTs[5] = {0};
  txTime.getTimestamp(txTs);

  tx_resp_msg[2] = (n > 2) ? poll[2] : 0;
  memcpy(tx_resp_msg + 10, rxTs, 4);
  memcpy(tx_resp_msg + 14, txTs, 4);
  DW1000.setData(tx_resp_msg, 18);
  DW1000.startTransmit();

  /* Time from poll RX to the TX start command. Past REPLY_DLY_US the DW1000
   * would hold the frame for a whole 17 s timer wrap. */
  DW1000Time now;
  DW1000.getSystemTimestamp(now);
  int64_t prep = (now - rx_time).getTimestamp();
  if (prep < 0) {
    prep += DW1000Time::TIME_OVERFLOW;
  }
  uint32_t prep_us = (uint32_t)(prep / 63898LL);  /* 63897.6 ticks (15.65 ps) per us */
  if (prep_us > max_prep_us) {
    max_prep_us = prep_us;
  }
  if (prep_us >= REPLY_DLY_US) {
    n_late++;
    DW1000.idle();
    startListen();
    return;
  }

  uint32_t t0 = millis();
  bool done = false;
  while ((millis() - t0) < 30) {
    DW1000.readSystemEventStatusRegister();
    if (DW1000.isTransmitDone()) {
      done = true;
      break;
    }
  }
  if (done) {
    n_resp++;
    DW1000.clearTransmitStatus();
  } else {
    n_tx_stuck++;
    DW1000.idle();
  }
  if (verbose) {
    char line[64];
    snprintf(line, sizeof(line), "RESP prep %lu us", (unsigned long)prep_us);
    say(line);
  }
  startListen();
}

static void print_stats() {
  char line[128];
  snprintf(line, sizeof(line), "STATS resp %lu  late %lu  tx stuck %lu  rx err %lu  other %lu",
           (unsigned long)n_resp, (unsigned long)n_late, (unsigned long)n_tx_stuck,
           (unsigned long)n_rx_err, (unsigned long)n_other);
  say(line);
  snprintf(line, sizeof(line), "  reply prep max %lu us (reply delay %u us)  DEV_ID %08lX",
           (unsigned long)max_prep_us, (unsigned)REPLY_DLY_US, (unsigned long)read_dev_id());
  say(line);
  snprintf(line, sizeof(line), "  last poll RX %d dBm  first path %d dBm",
           (int)last_rx_dbm, (int)last_fp_dbm);
  say(line);
  /* RXPRD (bit 8) / RXSFDD (bit 9) show the radio hears preambles even when no
   * frame passes; SYS_STATE byte 2 is the RX state (0 = idle). */
  byte st[5], ss[4];
  DW1000.readBytes(SYS_STATUS, NO_SUB, st, 5);
  DW1000.readBytes(0x19, NO_SUB, ss, 4);
  snprintf(line, sizeof(line), "  SYS_STATUS %02X%02X%02X%02X%02X  SYS_STATE %02X%02X%02X%02X",
           st[4], st[3], st[2], st[1], st[0], ss[3], ss[2], ss[1], ss[0]);
  say(line);
  if (last_other_len) {
    int k = snprintf(line, sizeof(line), "  last other frame len %u:", last_other_len);
    for (uint16_t i = 0; i < last_other_len && i < sizeof(last_other) && k < 120; i++) {
      k += snprintf(line + k, sizeof(line) - k, " %02X", last_other[i]);
    }
    say(line);
  }
}

static void handle_usb() {
  static bool host_open = false;
  bool dtr = Serial.dtr();
  if (dtr && !host_open) {
    delay(100);
    banner();
  }
  host_open = dtr;
  while (Serial.available()) {
    int c = Serial.read();
    if (c == 'v') {
      verbose = !verbose;
      say(verbose ? "verbose on" : "verbose off");
    } else if (c == 'z') {
      n_resp = n_late = n_rx_err = n_other = n_tx_stuck = max_prep_us = 0;
      last_other_len = 0;
      say("stats zeroed");
    } else if (c == 'r') {
      DW1000.idle();
      board_check();
      startListen();
    }
  }
}

void loop() {
  static uint32_t last_stats = 0;
  static uint32_t last_poll = 0;
  handle_usb();
  if (millis() - last_stats >= STATS_EVERY_MS) {
    last_stats = millis();
    print_stats();
  }

  /* IRQ pin first; also read the status every 5 ms in case the pin is not wired. */
  if (digitalRead(PIN_IRQ) == HIGH || (millis() - last_poll) >= 5) {
    last_poll = millis();
    DW1000.handleInterrupt();
  }
  if (rx_failed) {
    rx_failed = false;
    n_rx_err++;
    startListen();
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
    n_other++;
    last_other_len = n;
    memcpy(last_other, poll, n < sizeof(last_other) ? n : sizeof(last_other));
    startListen();
  }
}
