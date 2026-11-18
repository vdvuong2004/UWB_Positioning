#include "dw3000.h"
#include <Wire.h>
#include <math.h>

#define APP_NAME "DEBUG IMU v1.4"
/*
 * IMU test: custom STM32F103RCT6 tag board + MPU6050 (or MPU6500/9250) on I2C1.
 *   MPU SCL -> PB6, MPU SDA -> PB7, VCC 3V3, GND, AD0 open/low = 0x68 (high = 0x69).
 * Steps: checks that both lines idle high (pull-ups), scans the bus, reads WHO_AM_I,
 * configures the MPU and prints at RATES_HZ:
 *   IMU <t_ms> <ax> <ay> <az> <gx> <gy> <gz>      accel in g, gyro in deg/s
 * plus every 5 s a STATS line (rate, read errors, |a| mean, gyro mean/std),
 * which shows the noise and bias with the board lying still.
 * Includes dw3000.h only for the board clock setup (72 MHz from the crystal).
 *
 * Board: Generic F103RCTx, USB support: CDC. Log on the board's USB port.
 * Commands: f = print rate (50/100/200/10 Hz), c = gyro bias calibration (keep still),
 *           a = 6-face accel calibration (prints ACAL off/scale),
 *           i = IMU lines on/off, s = rescan + re-init, w = raw register values on/off.
 */

#if !defined(ARDUINO_ARCH_STM32) || !defined(STM32F103xE)
#error "Select Generic STM32F1 series, part Generic F103RCTx."
#endif
#if !defined(USBCON) || !defined(USBD_USE_CDC)
#error "Tools > USB support: CDC (generic 'Serial' supersede U(S)ART)."
#endif

#define PIN_IMU_SCL PB6
#define PIN_IMU_SDA PB7
#define I2C_HZ 400000

/* MPU-60x0 / 65x0 registers. */
#define REG_SMPLRT_DIV 0x19
#define REG_CONFIG 0x1A
#define REG_GYRO_CONFIG 0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_ACCEL_XOUT_H 0x3B
#define REG_PWR_MGMT_1 0x6B
#define REG_WHO_AM_I 0x75

/* +-4 g, +-500 deg/s: a robot does not exceed them, finer steps than the larger ranges. */
#define ACCEL_CFG 0x08
#define ACCEL_LSB_PER_G 8192.0f
#define GYRO_CFG 0x08
#define GYRO_LSB_PER_DPS 65.5f
/* DLPF 44 Hz (accel) / 42 Hz (gyro), 1 kHz internal rate: cuts motor vibration. */
#define DLPF_CFG 0x03

/* Above every function: the Arduino prototype generator puts its prototypes here. */
struct sample_t {
  int16_t raw[7]; /* ax ay az temp gx gy gz */
  float a[3];     /* g */
  float g[3];     /* deg/s, bias removed */
  float temp_c;
};

static const uint16_t RATES_HZ[] = {50, 100, 200, 10};
#define STATS_PERIOD_MS 5000
#define CAL_SAMPLES 500

static uint8_t mpu_addr = 0;
static uint8_t who = 0;
static bool mpu_ok = false;
static uint16_t rate_hz = 50;
static bool print_imu = true;
static bool print_raw = false;
static float gyro_bias[3] = {0, 0, 0};
/* Accel correction a = (a_read - off) / scale, per axis, from the 6-face calibration ('a').
 * Board B, 6-face run 2026-11-18: Z reads +1.46 / -0.54 g (+0.46 g offset, scale OK).
 * The X faces were ~12 deg tilted, so the X scale (0.98) may read ~2% low. */
static float acc_off[3] = {0.0249f, -0.0080f, 0.4571f};
static float acc_scale[3] = {0.9802f, 0.9938f, 1.0076f};

/* Stats window. */
static uint32_t win_start = 0;
static uint32_t n_read = 0, n_err = 0;
static float a_norm_sum = 0;
static float g_sum[3], g_sum_sq[3];

static void say(const char *msg)
{
  SerialUSB.write(msg);
  SerialUSB.write("\r\n");
}

/* Appends v with dec decimals (newlib-nano printf has no %f). Returns chars written. */
static int put_fixed(char *s, int n, float v, uint8_t dec)
{
  if (n <= 0) {
    return 0;
  }
  long p = 1;
  for (uint8_t k = 0; k < dec; k++) {
    p *= 10;
  }
  long c = lroundf(v * p);
  unsigned long a = c < 0 ? -c : c;
  int len = snprintf(s, n, "%s%lu.%0*lu", c < 0 ? "-" : "", a / p, (int)dec, a % p);
  return len < n ? len : n - 1;
}

static void wait_usb_host(void)
{
  uint32_t start = millis();
  while (!SerialUSB && (millis() - start < 5000)) {
    delay(10);
  }
  delay(100);
}

static bool reg_write(uint8_t reg, uint8_t val)
{
  Wire.beginTransmission(mpu_addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static bool reg_read(uint8_t reg, uint8_t *buf, uint8_t n)
{
  Wire.beginTransmission(mpu_addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  if (Wire.requestFrom(mpu_addr, n) != n) {
    return false;
  }
  for (uint8_t k = 0; k < n; k++) {
    buf[k] = Wire.read();
  }
  return true;
}

static const char *who_name(uint8_t w)
{
  switch (w) {
    case 0x68: return "MPU6050";
    case 0x70: return "MPU6500";
    case 0x71: return "MPU9250";
    case 0x73: return "MPU9255";
    case 0x72: return "MPU6050 clone";
    case 0x98: return "MPU6050 clone (0x98)";
    default: return "unknown";
  }
}

/* Lines must idle high before the I2C peripheral takes the pins: a low line is a
 * missing pull-up (the GY-521 has 2.2k-4.7k on board), a short, or no module power. */
static void check_lines(void)
{
  pinMode(PIN_IMU_SCL, INPUT);
  pinMode(PIN_IMU_SDA, INPUT);
  delay(2);
  int scl = digitalRead(PIN_IMU_SCL);
  int sda = digitalRead(PIN_IMU_SDA);
  char line[80];
  snprintf(line, sizeof(line), "lines idle: SCL PB6 %s, SDA PB7 %s", scl ? "high OK" : "LOW",
           sda ? "high OK" : "LOW");
  say(line);
  if (!scl || !sda) {
    say("  -> check module VCC/GND, pull-ups on SCL/SDA, wiring");
  }
}

static void scan_and_init(void)
{
  static bool wire_on = false;
  mpu_ok = false;
  mpu_addr = 0;
  if (wire_on) {
    Wire.end(); /* give the pins back so check_lines() reads them */
  }
  check_lines();
  wire_on = true;
  Wire.setSCL(PIN_IMU_SCL);
  Wire.setSDA(PIN_IMU_SDA);
  Wire.begin();
  Wire.setClock(I2C_HZ);

  char line[96];
  int len = snprintf(line, sizeof(line), "i2c scan:");
  uint8_t found = 0;
  for (uint8_t a = 0x08; a < 0x78; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      found++;
      if (len < (int)sizeof(line)) {
        len += snprintf(line + len, sizeof(line) - len, " 0x%02X", a);
      }
      if ((a == 0x68 || a == 0x69) && mpu_addr == 0) {
        mpu_addr = a;
      }
    }
  }
  if (!found) {
    snprintf(line + len, sizeof(line) - len, " nothing");
  }
  say(line);
  if (!mpu_addr) {
    say("MPU NOT FOUND at 0x68/0x69");
    return;
  }

  if (!reg_read(REG_WHO_AM_I, &who, 1)) {
    say("WHO_AM_I read failed");
    return;
  }
  snprintf(line, sizeof(line), "MPU at 0x%02X WHO_AM_I 0x%02X (%s)", mpu_addr, who, who_name(who));
  say(line);

  /* Reset, then wake on the gyro X PLL (steadier clock than the 8 MHz RC). */
  reg_write(REG_PWR_MGMT_1, 0x80);
  delay(100);
  bool ok = reg_write(REG_PWR_MGMT_1, 0x01);
  delay(10);
  ok = ok && reg_write(REG_SMPLRT_DIV, 0x00);
  ok = ok && reg_write(REG_CONFIG, DLPF_CFG);
  ok = ok && reg_write(REG_GYRO_CONFIG, GYRO_CFG);
  ok = ok && reg_write(REG_ACCEL_CONFIG, ACCEL_CFG);
  uint8_t chk[4];
  ok = ok && reg_read(REG_CONFIG, chk, 3);
  if (!ok || chk[0] != DLPF_CFG || chk[1] != GYRO_CFG || chk[2] != ACCEL_CFG) {
    snprintf(line, sizeof(line), "MPU CONFIG FAILED (read back %02X %02X %02X)", chk[0], chk[1], chk[2]);
    say(line);
    return;
  }
  /* The first samples after wake-up are garbage (gyro read -65 dps at rest). */
  delay(100);
  say("MPU ready: +-4 g, +-500 dps, DLPF 44 Hz");
  mpu_ok = true;
}

static void banner(void)
{
  char line[48];
  say(APP_NAME);
  say(dw3000_clock_from_hse ? "CLOCK HSE 72 MHz" : "CLOCK HSI 48 MHz (crystal failed)");
  say("commands: f = rate, c = gyro calibration, a = accel 6-face calibration, i = IMU lines on/off, s = rescan, w = raw on/off");
  snprintf(line, sizeof(line), "rate %u Hz", rate_hz);
  say(line);
  print_acal();
}

static void reset_window(void)
{
  win_start = millis();
  n_read = n_err = 0;
  a_norm_sum = 0;
  for (uint8_t k = 0; k < 3; k++) {
    g_sum[k] = g_sum_sq[k] = 0;
  }
}

static bool imu_read(sample_t *s)
{
  uint8_t b[14];
  if (!reg_read(REG_ACCEL_XOUT_H, b, 14)) {
    return false;
  }
  for (uint8_t k = 0; k < 7; k++) {
    s->raw[k] = (int16_t)((b[2 * k] << 8) | b[2 * k + 1]);
  }
  for (uint8_t k = 0; k < 3; k++) {
    s->a[k] = (s->raw[k] / ACCEL_LSB_PER_G - acc_off[k]) / acc_scale[k];
    s->g[k] = s->raw[4 + k] / GYRO_LSB_PER_DPS - gyro_bias[k];
  }
  /* Temperature formula differs between the 6050 and the 6500 family. */
  s->temp_c = (who == 0x68 || who == 0x72 || who == 0x98) ? s->raw[3] / 340.0f + 36.53f
                                                          : s->raw[3] / 333.87f + 21.0f;
  return true;
}

static void calibrate_gyro(void)
{
  say("gyro calibration: keep the board still...");
  float sum[3] = {0, 0, 0};
  uint16_t n = 0;
  float saved[3] = {gyro_bias[0], gyro_bias[1], gyro_bias[2]};
  gyro_bias[0] = gyro_bias[1] = gyro_bias[2] = 0;
  for (uint16_t i = 0; i < CAL_SAMPLES; i++) {
    sample_t s;
    if (imu_read(&s)) {
      for (uint8_t k = 0; k < 3; k++) {
        sum[k] += s.g[k];
      }
      n++;
    }
    delay(2);
  }
  if (n < CAL_SAMPLES / 2) {
    for (uint8_t k = 0; k < 3; k++) {
      gyro_bias[k] = saved[k];
    }
    say("calibration failed (read errors)");
    return;
  }
  char line[80];
  int len = snprintf(line, sizeof(line), "gyro bias dps");
  for (uint8_t k = 0; k < 3; k++) {
    gyro_bias[k] = sum[k] / n;
    len += snprintf(line + len, sizeof(line) - len, " ");
    len += put_fixed(line + len, sizeof(line) - len, gyro_bias[k], 3);
  }
  say(line);
  reset_window();
}

static void print_acal(void)
{
  char line[112];
  int len = snprintf(line, sizeof(line), "ACAL off");
  for (uint8_t k = 0; k < 3; k++) {
    len += snprintf(line + len, sizeof(line) - len, " ");
    len += put_fixed(line + len, sizeof(line) - len, acc_off[k], 4);
  }
  len += snprintf(line + len, sizeof(line) - len, " scale");
  for (uint8_t k = 0; k < 3; k++) {
    len += snprintf(line + len, sizeof(line) - len, " ");
    len += put_fixed(line + len, sizeof(line) - len, acc_scale[k], 4);
  }
  say(line);
}

/* 6-face accel calibration. Put the board still with each axis up and down in any
 * order; a face is taken after 1 s without motion. Per axis, from the two faces:
 *   off = (up + down) / 2,  scale = (up - down) / 2   (in g, uncorrected readings).
 * Blocks until done; q aborts. */
#define ACAL_WIN 100           /* samples at 100 Hz = 1 s still */
/* Held in a hand the accel std is 0.015-0.03 g; the mean of 100 samples is still
 * within ~3 mg. A slow turn has a small gyro std but a gyro mean, so check both. */
#define ACAL_STILL_G 0.03f     /* accel std per axis */
#define ACAL_STILL_DPS 6.0f    /* gyro std per axis */
#define ACAL_TURN_DPS 8.0f     /* gyro mean per axis (bias here ~1.5 dps) */
#define ACAL_TILT 0.25f        /* other axes / main axis: ~14 deg */
static void calibrate_accel(void)
{
  static const char *FACE[6] = {"+X", "-X", "+Y", "-Y", "+Z", "-Z"};
  float face_g[6][3];
  uint8_t have = 0; /* bit per face */
  int8_t last_face = -1;
  uint32_t n_fail = 0;
  char line[112];
  say("accel 6-face calibration: lay the board still with +X,-X,+Y,-Y,+Z,-Z up (any order), q = abort");

  while (have != 0x3F) {
    float sa[3] = {0, 0, 0}, sa2[3] = {0, 0, 0}, sg[3] = {0, 0, 0}, sg2[3] = {0, 0, 0};
    uint16_t n = 0;
    while (n < ACAL_WIN) {
      while (SerialUSB.available()) {
        char c = SerialUSB.read();
        if (c == 'q' || c == 'Q') {
          say("accel calibration aborted");
          return;
        }
      }
      sample_t s;
      if (imu_read(&s)) {
        for (uint8_t k = 0; k < 3; k++) {
          float a = s.raw[k] / ACCEL_LSB_PER_G;
          float g = s.raw[4 + k] / GYRO_LSB_PER_DPS;
          sa[k] += a;
          sa2[k] += a * a;
          sg[k] += g;
          sg2[k] += g * g;
        }
        n++;
      } else if (++n_fail % 100 == 0) {
        /* Otherwise a loose wire looks like a hang. */
        snprintf(line, sizeof(line), "read errors %lu (check SCL/SDA/GND wires)", (unsigned long)n_fail);
        say(line);
      }
      delay(10);
    }
    float m[3];
    float va_max = 0, vg_max = 0, mg_max = 0;
    for (uint8_t k = 0; k < 3; k++) {
      m[k] = sa[k] / n;
      float va = sa2[k] / n - m[k] * m[k];
      float mg = sg[k] / n;
      float vg = sg2[k] / n - mg * mg;
      va_max = va > va_max ? va : va_max;
      vg_max = vg > vg_max ? vg : vg_max;
      mg_max = fabsf(mg) > mg_max ? fabsf(mg) : mg_max;
    }
    if (va_max >= ACAL_STILL_G * ACAL_STILL_G || vg_max >= ACAL_STILL_DPS * ACAL_STILL_DPS ||
        mg_max >= ACAL_TURN_DPS) {
      /* Shows which limit stops a face from being taken. */
      int len = snprintf(line, sizeof(line), "moving: a std ");
      len += put_fixed(line + len, sizeof(line) - len, sqrtf(va_max), 3);
      len += snprintf(line + len, sizeof(line) - len, " g (max ");
      len += put_fixed(line + len, sizeof(line) - len, ACAL_STILL_G, 3);
      len += snprintf(line + len, sizeof(line) - len, ")  gyro std ");
      len += put_fixed(line + len, sizeof(line) - len, sqrtf(vg_max), 1);
      len += snprintf(line + len, sizeof(line) - len, " dps (max ");
      len += put_fixed(line + len, sizeof(line) - len, ACAL_STILL_DPS, 1);
      len += snprintf(line + len, sizeof(line) - len, ") mean ");
      len += put_fixed(line + len, sizeof(line) - len, mg_max, 1);
      len += snprintf(line + len, sizeof(line) - len, "  read");
      for (uint8_t k = 0; k < 3; k++) {
        len += snprintf(line + len, sizeof(line) - len, " ");
        len += put_fixed(line + len, sizeof(line) - len, m[k], 2);
      }
      say(line);
      last_face = -1;
      continue;
    }
    /* Axis and tilt from offset-corrected values: with the raw +0.46 g on Z every
     * X/Y face looked 25 deg tilted. The stored face values stay uncorrected. */
    float c[3];
    for (uint8_t k = 0; k < 3; k++) {
      c[k] = m[k] - acc_off[k];
    }
    uint8_t ax = 0;
    for (uint8_t k = 1; k < 3; k++) {
      if (fabsf(c[k]) > fabsf(c[ax])) {
        ax = k;
      }
    }
    bool level = true;
    for (uint8_t k = 0; k < 3; k++) {
      level = level && (k == ax || fabsf(c[k]) < ACAL_TILT * fabsf(c[ax]));
    }
    int8_t face = ax * 2 + (c[ax] < 0);
    if (face == last_face) {
      continue; /* say each face once */
    }
    last_face = face;
    int len = snprintf(line, sizeof(line), "%s up, read", FACE[face]);
    for (uint8_t k = 0; k < 3; k++) {
      len += snprintf(line + len, sizeof(line) - len, " ");
      len += put_fixed(line + len, sizeof(line) - len, m[k], 3);
    }
    if (!level) {
      snprintf(line + len, sizeof(line) - len, "  TILTED, not taken");
    } else if (have & (1 << face)) {
      snprintf(line + len, sizeof(line) - len, "  (already have it)");
    } else {
      have |= 1 << face;
      for (uint8_t k = 0; k < 3; k++) {
        face_g[face][k] = m[k];
      }
      len += snprintf(line + len, sizeof(line) - len, "  taken, left:");
      for (uint8_t f = 0; f < 6; f++) {
        if (!(have & (1 << f))) {
          len += snprintf(line + len, sizeof(line) - len, " %s", FACE[f]);
        }
      }
    }
    say(line);
  }

  for (uint8_t k = 0; k < 3; k++) {
    float up = face_g[2 * k][k], down = face_g[2 * k + 1][k];
    acc_off[k] = 0.5f * (up + down);
    acc_scale[k] = 0.5f * (up - down);
  }
  say("accel calibration done (copy this line into the sketch defaults):");
  print_acal();
  reset_window();
}

static void print_stats(sample_t *last)
{
  char line[128];
  uint32_t elapsed = millis() - win_start;
  uint32_t hz_x10 = elapsed ? n_read * 10000UL / elapsed : 0;
  int len = snprintf(line, sizeof(line), "STATS %lu.%lu Hz  errors %lu  |a| ", (unsigned long)(hz_x10 / 10),
                     (unsigned long)(hz_x10 % 10), (unsigned long)n_err);
  len += put_fixed(line + len, sizeof(line) - len, n_read ? a_norm_sum / n_read : 0, 3);
  len += snprintf(line + len, sizeof(line) - len, " g  gyro mean/std dps");
  for (uint8_t k = 0; k < 3 && n_read; k++) {
    float m = g_sum[k] / n_read;
    float v = g_sum_sq[k] / n_read - m * m;
    len += snprintf(line + len, sizeof(line) - len, " ");
    len += put_fixed(line + len, sizeof(line) - len, m, 2);
    len += snprintf(line + len, sizeof(line) - len, "/");
    len += put_fixed(line + len, sizeof(line) - len, sqrtf(v > 0 ? v : 0), 2);
  }
  if (last) {
    len += snprintf(line + len, sizeof(line) - len, "  temp ");
    put_fixed(line + len, sizeof(line) - len, last->temp_c, 1);
  }
  say(line);
  reset_window();
}

static void handle_usb(void)
{
  static bool host_open = SerialUSB.dtr();
  bool dtr = SerialUSB.dtr();
  if (dtr && !host_open) {
    delay(100);
    banner();
    scan_and_init();
  }
  host_open = dtr;

  while (SerialUSB.available()) {
    char c = SerialUSB.read();
    char line[32];
    if (c == 'f' || c == 'F') {
      uint8_t n = sizeof(RATES_HZ) / sizeof(RATES_HZ[0]);
      uint8_t k = 0;
      while (k < n && RATES_HZ[k] != rate_hz) {
        k++;
      }
      rate_hz = RATES_HZ[(k + 1) % n];
      snprintf(line, sizeof(line), "rate %u Hz", rate_hz);
      say(line);
      reset_window();
    } else if (c == 'c' || c == 'C') {
      if (mpu_ok) {
        calibrate_gyro();
      }
    } else if (c == 'a' || c == 'A') {
      if (mpu_ok) {
        calibrate_accel();
      }
    } else if (c == 'i' || c == 'I') {
      print_imu = !print_imu;
      say(print_imu ? "IMU lines on" : "IMU lines off");
    } else if (c == 'w' || c == 'W') {
      print_raw = !print_raw;
      say(print_raw ? "raw on" : "raw off");
    } else if (c == 's' || c == 'S') {
      scan_and_init();
      reset_window();
    }
  }
}

void setup()
{
  UART_init();
  wait_usb_host();
  banner();
  scan_and_init();
  reset_window();
}

void loop()
{
  static uint32_t next_us = micros();
  static uint32_t last_retry = 0;
  static sample_t last;
  static bool have_last = false;

  handle_usb();

  if (!mpu_ok) {
    /* Retry once a second so a loose wire can be fixed without a reset. */
    if (millis() - last_retry > 1000) {
      last_retry = millis();
      scan_and_init();
      reset_window();
    }
    return;
  }

  if ((int32_t)(micros() - next_us) < 0) {
    return;
  }
  next_us += 1000000UL / rate_hz;
  if ((int32_t)(micros() - next_us) > 0) {
    next_us = micros(); /* fell behind (USB stall): do not burst */
  }

  sample_t s;
  if (!imu_read(&s)) {
    n_err++;
    if (n_err > 20 && n_read == 0) {
      say("IMU READ FAILING, re-init");
      scan_and_init();
      reset_window();
    }
    return;
  }
  n_read++;
  a_norm_sum += sqrtf(s.a[0] * s.a[0] + s.a[1] * s.a[1] + s.a[2] * s.a[2]);
  for (uint8_t k = 0; k < 3; k++) {
    g_sum[k] += s.g[k];
    g_sum_sq[k] += s.g[k] * s.g[k];
  }
  last = s;
  have_last = true;

  if (print_imu) {
    char line[112];
    int len = snprintf(line, sizeof(line), "IMU %lu", (unsigned long)millis());
    for (uint8_t k = 0; k < 3; k++) {
      len += snprintf(line + len, sizeof(line) - len, " ");
      len += put_fixed(line + len, sizeof(line) - len, s.a[k], 3);
    }
    for (uint8_t k = 0; k < 3; k++) {
      len += snprintf(line + len, sizeof(line) - len, " ");
      len += put_fixed(line + len, sizeof(line) - len, s.g[k], 2);
    }
    if (print_raw) {
      snprintf(line + len, sizeof(line) - len, "  raw %d %d %d %d %d %d", s.raw[0], s.raw[1], s.raw[2],
               s.raw[4], s.raw[5], s.raw[6]);
    }
    say(line);
  }

  if (millis() - win_start >= STATS_PERIOD_MS) {
    print_stats(have_last ? &last : NULL);
  }
}
