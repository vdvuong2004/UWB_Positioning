#include <SPI.h>
#include <DW1000.h>

#define APP_NAME "UWB ANCHOR DW1000 v2.1"

/*
 * UWB anchor (SS-TWR responder) with a DWM1000. Answers polls addressed to
 * ANCHOR_ID from uwb_tag (Dw3000 library examples). Give every anchor its own
 * ID and list it in the tag's ANCHORS[] table with PWR_DW1000.
 *
 * Boards:
 *   Custom STM32F103RCT6 board, DWM1000 on the module header (J7/J8):
 *     "Generic STM32F1 series", part "Generic F103RCTx",
 *     Tools > USB support: "CDC (generic 'Serial' supersede U(S)ART)". Log on USB.
 *     SCK PB3, MISO PB4, MOSI PB5, CS PA15, IRQ PC12, RST PD2 (bit-banged SPI).
 *   ESP32-C3 SuperMini (test station): "ESP32C3 Dev Module".
 *     SCK 4, MISO 5, MOSI 6, CS 7, IRQ 3, RST 2. With USB CDC On Boot disabled
 *     the log goes to the USB console through printf.
 *
 * Boot runs a board check (DEV_ID x1000, temperature, supply), then listens.
 * Every 10 s: STATS with replies, late replies, RX errors, other frames, the
 * worst reply prep time and a DEV_ID re-read (SPI health).
 * Commands: v = verbose on/off, z = zero stats, r = rerun the board check.
 *
 * Protocol: see uwb_tag. Change the tag and both anchor sketches together.
 */

/* Unique per anchor. Can also be set with -DANCHOR_ID=n. */
#ifndef ANCHOR_ID
#define ANCHOR_ID 3
#endif

#if defined(ARDUINO_ARCH_STM32)
#if !defined(USBCON) || !defined(USBD_USE_CDC)
#error "Tools > USB support: CDC (generic 'Serial' supersede U(S)ART)"
#endif
const uint8_t PIN_RST = PD2;
const uint8_t PIN_IRQ = PC12;
const uint8_t PIN_SCK = PB3;
const uint8_t PIN_MISO = PB4;
const uint8_t PIN_MOSI = PB5;
const uint8_t PIN_SS = PA15;
#elif defined(ARDUINO_ARCH_ESP32)
const uint8_t PIN_RST = 2;
const uint8_t PIN_IRQ = 3;
const uint8_t PIN_SCK = 4;
const uint8_t PIN_MISO = 5;
const uint8_t PIN_MOSI = 6;
const uint8_t PIN_SS = 7;
#else
#error "Supported boards: custom STM32F103RCT6 board, ESP32-C3."
#endif

/* The tag expects this exact delay. Reply prep time is measured and printed;
 * past REPLY_DELAY_US the DW1000 would hold the frame for a 17 s timer wrap. */
#define REPLY_DELAY_US 3000
#define ANTENNA_DELAY 16436
#define STATS_PERIOD_MS 10000

#define FC_LO 0x41
#define FC_HI 0x88
#define PAN_LO 0xCA
#define PAN_HI 0xDE
#define FUNC_POLL 0xE0
#define FUNC_RESP 0xE1
#define IDX_SEQ 2
#define IDX_DST 5
#define IDX_SRC 7
#define IDX_FUNC 9
#define IDX_POLL_RX_TS 10
#define IDX_RESP_TX_TS 14
#define POLL_LEN 12
/* The DW1000 library counts the 2 CRC bytes outside the data. */
#define RESP_DATA_LEN 18

static byte resp_msg[RESP_DATA_LEN] = {FC_LO, FC_HI, 0, PAN_LO, PAN_HI, 0, 0,
                                       ANCHOR_ID & 0xFF, ANCHOR_ID >> 8, FUNC_RESP};

volatile boolean received = false;
volatile boolean rx_failed = false;

static bool verbose = false;
static uint32_t n_resp = 0;
static uint32_t n_late = 0;
static uint32_t n_rx_err = 0;
static uint32_t n_other = 0;
static uint32_t n_tx_stuck = 0;
static uint32_t n_rearm = 0;
static uint32_t max_prep_us = 0;

static void say(const char *text) {
#if defined(ARDUINO_ARCH_ESP32) && !ARDUINO_USB_CDC_ON_BOOT
  printf("%s\r\n", text);
#else
  if (Serial) {
    Serial.println(text);
  }
#endif
}

static uint32_t read_dev_id() {
  byte id[4];
  DW1000.readBytes(DEV_ID, NO_SUB, id, 4);
  return ((uint32_t)id[3] << 24) | ((uint32_t)id[2] << 16) | ((uint32_t)id[1] << 8) | id[0];
}

static void start_listen() {
  DW1000.newReceive();
  DW1000.startReceive();
}

static void handle_received() {
  received = true;
}

/* Without this a bad frame (other UWB traffic nearby) stops the receiver for good. */
static void handle_receive_failed() {
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
  DW1000.setAntennaDelay(ANTENNA_DELAY);
  DW1000.commitConfiguration();
}

static void banner() {
  char line[48];
  say(APP_NAME);
  snprintf(line, sizeof(line), "anchor id %u  reply delay %u us", ANCHOR_ID, (unsigned)REPLY_DELAY_US);
  say(line);
  say("commands: v = verbose on/off, z = zero stats, r = rerun board check");
}

void setup() {
  Serial.begin(115200);
#if defined(ARDUINO_ARCH_STM32)
  /* Logs printed before the host opens the port are lost, so wait for DTR. */
  uint32_t start = millis();
  while (!Serial && (millis() - start < 5000)) {
    delay(10);
  }
  delay(100);
#else
  delay(200);
#endif
  banner();

  DW1000.setSpiPins(PIN_SCK, PIN_MISO, PIN_MOSI);
  DW1000.begin(PIN_IRQ, PIN_RST);
  /* Poll the IRQ pin from loop(); SPI inside the ISR would race loop(). */
  detachInterrupt(digitalPinToInterrupt(PIN_IRQ));
#if defined(ARDUINO_ARCH_ESP32)
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, -1);
#endif
  DW1000.select(PIN_SS);
  board_check();

  configure();
  DW1000.attachReceivedHandler(handle_received);
  DW1000.attachReceiveFailedHandler(handle_receive_failed);
  start_listen();
  say("listening");
}

static bool is_poll_for_me(const byte *f, uint16_t n) {
  /* getDataLength() leaves out the 2 CRC bytes. */
  return n == POLL_LEN - 2 && f[0] == FC_LO && f[1] == FC_HI && f[3] == PAN_LO && f[4] == PAN_HI &&
         f[IDX_DST] == (ANCHOR_ID & 0xFF) && f[IDX_DST + 1] == (ANCHOR_ID >> 8) && f[IDX_FUNC] == FUNC_POLL;
}

static void reply_to_poll(const byte *poll) {
  DW1000Time rx_time;
  DW1000.getReceiveTimestamp(rx_time);
  byte rx_ts[5] = {0};
  rx_time.getTimestamp(rx_ts);

  DW1000.newTransmit();
  DW1000Time tx_time = DW1000.setDelay(DW1000Time((int32_t)REPLY_DELAY_US, DW1000Time::MICROSECONDS));
  byte tx_ts[5] = {0};
  tx_time.getTimestamp(tx_ts);

  resp_msg[IDX_SEQ] = poll[IDX_SEQ];
  resp_msg[IDX_DST] = poll[IDX_SRC];
  resp_msg[IDX_DST + 1] = poll[IDX_SRC + 1];
  memcpy(resp_msg + IDX_POLL_RX_TS, rx_ts, 4);
  memcpy(resp_msg + IDX_RESP_TX_TS, tx_ts, 4);
  DW1000.setData(resp_msg, RESP_DATA_LEN);
  DW1000.startTransmit();

  /* Poll RX to the TX start command, on the DW1000 clock. */
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
  if (prep_us >= REPLY_DELAY_US) {
    n_late++;
    DW1000.idle();
    start_listen();
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
    char line[40];
    snprintf(line, sizeof(line), "RESP prep %lu us", (unsigned long)prep_us);
    say(line);
  }
  start_listen();
}

static void print_stats() {
  static uint32_t last = 0;
  if (millis() - last < STATS_PERIOD_MS) {
    return;
  }
  last = millis();
  char line[112];
  snprintf(line, sizeof(line), "STATS resp %lu  late %lu  tx stuck %lu  rx err %lu  other %lu  rearm %lu",
           (unsigned long)n_resp, (unsigned long)n_late, (unsigned long)n_tx_stuck,
           (unsigned long)n_rx_err, (unsigned long)n_other, (unsigned long)n_rearm);
  say(line);
  snprintf(line, sizeof(line), "  reply prep max %lu us (reply delay %u us)  DEV_ID %08lX",
           (unsigned long)max_prep_us, (unsigned)REPLY_DELAY_US, (unsigned long)read_dev_id());
  say(line);
}

static void handle_console() {
#if defined(ARDUINO_ARCH_STM32)
  /* Upload resets the board and drops the COM port. Show the banner again
   * whenever a terminal (re)opens it. */
  static bool host_open = false;
  bool dtr = Serial.dtr();
  if (dtr && !host_open) {
    delay(100);
    banner();
  }
  host_open = dtr;
#endif
  while (Serial.available()) {
    int c = Serial.read();
    if (c == 'v') {
      verbose = !verbose;
      say(verbose ? "verbose on" : "verbose off");
    } else if (c == 'z') {
      n_resp = n_late = n_rx_err = n_other = n_tx_stuck = n_rearm = max_prep_us = 0;
      say("stats zeroed");
    } else if (c == 'r') {
      DW1000.idle();
      board_check();
      start_listen();
    }
  }
}

/* Re-arm RX if the receiver went idle for any other reason. SYS_STATE bits
 * 8..12 are the RX state, 0 = idle. */
static void rx_watchdog() {
  static uint32_t last = 0;
  if (millis() - last < 50) {
    return;
  }
  last = millis();
  byte ss[4];
  DW1000.readBytes(0x19, NO_SUB, ss, 4);
  /* IRQ high means a frame just finished: leave it for handleInterrupt(). */
  if ((ss[1] & 0x1F) == 0 && digitalRead(PIN_IRQ) == LOW) {
    n_rearm++;
    start_listen();
  }
}

void loop() {
  static uint32_t last_poll = 0;
  handle_console();
  print_stats();

  /* IRQ pin first; also read the status every 5 ms in case the pin is not wired. */
  if (digitalRead(PIN_IRQ) == HIGH || (millis() - last_poll) >= 5) {
    last_poll = millis();
    DW1000.handleInterrupt();
  }
  if (rx_failed) {
    rx_failed = false;
    n_rx_err++;
    start_listen();
  }
  if (!received) {
    rx_watchdog();
    return;
  }
  received = false;

  uint16_t n = DW1000.getDataLength();
  byte frame[16];
  DW1000.getData(frame, n < sizeof(frame) ? n : sizeof(frame));
  if (is_poll_for_me(frame, n)) {
    reply_to_poll(frame);
  } else {
    n_other++;
    start_listen();
  }
}
