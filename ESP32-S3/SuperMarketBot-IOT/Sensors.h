/* =====================================================================
 *  Sensors.h — Sensor Fusion: HC-SR04 (tầm gần) + TF-Luna (tầm xa)
 *    • USE_HC_SR04_HARDWARE = 1 → Khởi tạo + ping 4 góc SR04
 *    • USE_LIDAR_HARDWARE  = 1 → Khởi tạo + đọc 2 UART LiDAR
 *    • Cả hai chạy đồng thời, không loại trừ nhau.
 *
 *  Thứ tự gọi trong loop:
 *    sensorsPollLidar()  — LiDAR 100Hz, đọc UART nhanh
 *    sensorsPollUS()     — SR04 tuần tự 4 góc (chậm hơn)
 *
 *  API:
 *    sensorsInit()          — Init CẢ hai hệ thống
 *    sensorsPollUS()        — Ping HC-SR04 4 góc
 *    sensorsPollLidar()    — Đọc TF-Luna UART trước/sau
 *    sensorsLogBootSample() — Log boot CẢ hai hệ thống
 * =====================================================================*/
#ifndef SENSORS_H
#define SENSORS_H

#include "Config.h"
#include "SensorLayout.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

volatile uint32_t g_lunaRxBytes1 = 0;
volatile uint32_t g_lunaRxBytes2 = 0;
volatile uint32_t g_luna1LastOkMs = 0;
volatile uint32_t g_luna2LastOkMs = 0;
volatile uint32_t g_usPhyLastEchoMs[4] = {0, 0, 0, 0};

/* ==================== HC-SR04 (tầm gần, 4 góc) =================== */
#if USE_HC_SR04_HARDWARE
#include <NewPing.h>

static NewPing g_sonarLF(US_TRIG, US_ECHO_LF, US_PING_MAX_CM);
static NewPing g_sonarRL(US_TRIG, US_ECHO_RL, US_PING_MAX_CM);
static NewPing g_sonarRF(US_TRIG, US_ECHO_RF, US_PING_MAX_CM);
static NewPing g_sonarRR(US_TRIG, US_ECHO_RR, US_PING_MAX_CM);

/** Lọc nhiễu: giữ giá trị cũ nếu nhảy >35cm một nhịp. */
static int16_t s_usFiltered[4] = {
  (int16_t)US_PING_MAX_CM, (int16_t)US_PING_MAX_CM,
  (int16_t)US_PING_MAX_CM, (int16_t)US_PING_MAX_CM
};

static inline int16_t usFilterSample(uint8_t idx, int16_t raw) {
  if (raw <= 0) return s_usFiltered[idx];
  if (raw > (int16_t)US_PING_MAX_CM) raw = (int16_t)US_PING_MAX_CM;
  int16_t prev = s_usFiltered[idx];
  const bool vPrev = (prev > (int16_t)US_MIN_VALID_CM && prev < (int16_t)US_PING_MAX_CM);
  const bool vRaw  = (raw  > (int16_t)US_MIN_VALID_CM && raw  < (int16_t)US_PING_MAX_CM);
  if (vPrev && vRaw && abs((int)raw - (int)prev) > 35) {
    return prev;
  }
  s_usFiltered[idx] = raw;
  return raw;
}
#endif

/* ==================== Helper: nghỉ giữa ping ===================== */
inline void sensorsYieldMs(uint32_t ms) {
  const uint32_t m = ms ? ms : 1u;
  vTaskDelay(pdMS_TO_TICKS(m));
}

/* ==================== TF-Luna UART init ========================= */
inline void tflunaSendFrame(HardwareSerial &ser, const uint8_t *frame, size_t len) {
  if (len < 4) return;
  ser.write(frame, len);
  ser.flush();
  delay(25);
  while (ser.available()) (void)ser.read();
}

#if TFLUNA_SEND_INIT_CMD
inline void tflunaUartApplyDefaults(HardwareSerial &ser) {
  static const uint8_t kOutEn[] = {0x5A, 0x05, 0x07, 0x01, 0x67};
  static const uint8_t kFmtCm[] = {0x5A, 0x05, 0x05, 0x01, 0x65};
  static const uint8_t kSave[]  = {0x5A, 0x04, 0x11, 0x6F};

  tflunaSendFrame(ser, kOutEn, sizeof(kOutEn));
  tflunaSendFrame(ser, kFmtCm, sizeof(kFmtCm));

  uint8_t hz = (uint8_t)constrain((int)TFLUNA_SAMPLE_HZ, 1, 250);
  uint8_t rateCmd[6] = {0x5A, 0x06, 0x03, hz, 0x00, 0x00};
  uint16_t s = rateCmd[0];
  for (int i = 1; i < 5; i++) s += rateCmd[i];
  rateCmd[5] = (uint8_t)(s & 0xFFu);
  tflunaSendFrame(ser, rateCmd, sizeof(rateCmd));

  tflunaSendFrame(ser, kSave, sizeof(kSave));
  delay(40);
  while (ser.available()) (void)ser.read();
}
#else
inline void tflunaUartApplyDefaults(HardwareSerial &) {}
#endif

/* ==================== INIT: cả hai hệ thống ==================== */
inline void sensorsInit() {
#if USE_HC_SR04_HARDWARE
  pinMode(US_TRIG, OUTPUT);
  digitalWrite(US_TRIG, LOW);
  pinMode(US_ECHO_LF, INPUT);
  pinMode(US_ECHO_RL, INPUT);
  pinMode(US_ECHO_RF, INPUT);
  pinMode(US_ECHO_RR, INPUT);
  Serial.println(F("[US] HC-SR04 x4 (LF/RL/RF/RR) init — stop <30cm, OA tu 42cm."));
#endif

#if USE_LIDAR_HARDWARE
  Serial1.setRxBufferSize(1024);
  Serial2.setRxBufferSize(1024);
  Serial1.begin(LIDAR_BAUD, SERIAL_8N1, LIDAR_F_RX, LIDAR_F_TX);
  Serial2.begin(LIDAR_BAUD, SERIAL_8N1, LIDAR_B_RX, LIDAR_B_TX);
  delay(120);
  while (Serial1.available()) (void)Serial1.read();
  while (Serial2.available()) (void)Serial2.read();
#if TFLUNA_SEND_INIT_CMD
  tflunaUartApplyDefaults(Serial1);
  tflunaUartApplyDefaults(Serial2);
  Serial.println(F("[LiDAR] TF-Luna UART x2 (F/B) init."));
#endif
#endif

#if !USE_HC_SR04_HARDWARE && !USE_LIDAR_HARDWARE
  g_state.lidarFront = (int16_t)LIDAR_MAX_CM;
  g_state.lidarBack  = (int16_t)LIDAR_MAX_CM;
#endif
}

/* ==================== Gán SR04 → g_state (fusion helper) ========= */
inline void sensorsCommitPhyToState(const int16_t phy[4]) {
#if USE_HC_SR04_HARDWARE
  int16_t usSlot[4];
  for (int s = 0; s < 4; s++) {
    uint8_t p = g_mapUsSlot[s];
    if (p > 3) p = (uint8_t)s;
    usSlot[s] = phy[p];
  }
  g_state.usLF = usSlot[SLOT_LF];
  g_state.usLR = usSlot[SLOT_LR];
  g_state.usRF = usSlot[SLOT_RF];
  g_state.usRR = usSlot[SLOT_RR];
  g_state.usFront = (int16_t)min((int)g_state.usLF, (int)g_state.usRF);
  g_state.usBack = (int16_t)min((int)g_state.usLR, (int)g_state.usRR);
  g_state.usLeft = (int16_t)min((int)g_state.usLF, (int)g_state.usLR);
  g_state.usRight = (int16_t)min((int)g_state.usRF, (int)g_state.usRR);
#else
  (void)phy;
#endif
  g_state.usLastUpdateMs = millis();
}

/* ==================== TF-Luna frame parser ===================== */
/* Frame 9 byte: 0x59 0x59 DistL DistH  TempL TempH  CHK */
inline bool readTfLunaStream(HardwareSerial &ser, uint8_t *buf, uint8_t &idx,
                             volatile uint32_t &rxCount, int16_t &distCm) {
  bool gotFrame = false;
  while (ser.available()) {
    uint8_t b = (uint8_t)ser.read();
    rxCount++;
    if (idx == 0) {
      if (b == 0x59u) buf[idx++] = b;
    } else if (idx == 1) {
      if (b == 0x59u) buf[idx++] = b;
      else idx = 0;
    } else {
      buf[idx++] = b;
      if (idx >= 9) {
        uint16_t sum = 0;
        for (uint8_t i = 0; i < 8; i++) sum += buf[i];
        if ((sum & 0xFFu) == buf[8]) {
          distCm = (int16_t)(buf[2] | (buf[3] << 8));
          gotFrame = true;
        }
        idx = 0;
      }
    }
  }
  return gotFrame;
}

/* ==================== POLL LiDAR (tầm xa, trước/sau) ========== */
inline void sensorsPollLidar() {
#if !USE_LIDAR_HARDWARE
  return;
#endif
  static uint8_t buf1[9] = {0}, buf2[9] = {0};
  static uint8_t idx1 = 0, idx2 = 0;
  static int16_t rawD1 = (int16_t)LIDAR_MAX_CM;
  static int16_t rawD2 = (int16_t)LIDAR_MAX_CM;

  int16_t d;
  if (readTfLunaStream(Serial1, buf1, idx1, g_lunaRxBytes1, d)) {
    if (d < 0) d = 0;
    if (d > (int16_t)LIDAR_MAX_CM) d = (int16_t)LIDAR_MAX_CM;
    rawD1 = d;
    g_luna1LastOkMs = (uint32_t)millis();
    g_state.lidarLastUpdateMs = millis();
  }
  if (readTfLunaStream(Serial2, buf2, idx2, g_lunaRxBytes2, d)) {
    if (d < 0) d = 0;
    if (d > (int16_t)LIDAR_MAX_CM) d = (int16_t)LIDAR_MAX_CM;
    rawD2 = d;
    g_luna2LastOkMs = (uint32_t)millis();
    g_state.lidarLastUpdateMs = millis();
  }
  if (g_lidarFrontUart == 0) {
    g_state.lidarFront = rawD1;
    g_state.lidarBack = rawD2;
  } else {
    g_state.lidarFront = rawD2;
    g_state.lidarBack = rawD1;
  }
}

/* ==================== POLL SR04 (tầm gần, 4 góc) =============== */
inline void sensorsPollUS() {
#if !USE_HC_SR04_HARDWARE
  /* Fusion: g_state.us* giữ giá trị cũ từ lần poll cuối */
  g_state.usLeft  = (int16_t)US_PING_MAX_CM;
  g_state.usRight = (int16_t)US_PING_MAX_CM;
  g_state.usFront = (int16_t)US_PING_MAX_CM;
  g_state.usBack  = (int16_t)US_PING_MAX_CM;
  for (uint8_t i = 0; i < 4; i++) g_usPhyLastEchoMs[i] = 0;
  return;
#endif

  int16_t phy[4];
  int16_t r;

  r = (int16_t)g_sonarLF.ping_cm();
  phy[US_PHY_F] = usFilterSample(US_PHY_F, r);
  if (r > 0) g_usPhyLastEchoMs[US_PHY_F] = millis();
  sensorsYieldMs(US_INTER_PING_MS);

  r = (int16_t)g_sonarRL.ping_cm();
  phy[US_PHY_B] = usFilterSample(US_PHY_B, r);
  if (r > 0) g_usPhyLastEchoMs[US_PHY_B] = millis();
  sensorsYieldMs(US_INTER_PING_MS);

  r = (int16_t)g_sonarRF.ping_cm();
  phy[US_PHY_L] = usFilterSample(US_PHY_L, r);
  if (r > 0) g_usPhyLastEchoMs[US_PHY_L] = millis();
  sensorsYieldMs(US_INTER_PING_MS);

  r = (int16_t)g_sonarRR.ping_cm();
  phy[US_PHY_R] = usFilterSample(US_PHY_R, r);
  if (r > 0) g_usPhyLastEchoMs[US_PHY_R] = millis();

  sensorsCommitPhyToState(phy);
}

/* ==================== BOOT LOG cả hai hệ thống ================= */
inline void sensorsLogBootSample() {
  delay(150);

#if USE_HC_SR04_HARDWARE
  Serial.println(F("[US] Boot ping 4 goc (8 vong)..."));
  for (int i = 0; i < 8; i++) {
    sensorsPollUS();
    sensorsYieldMs(20);
  }
  Serial.printf(
      "  US LF:%d RL:%d RF:%d RR:%d | F:%d B:%d L:%d R:%d cm (stop<%d)\n",
      (int)g_state.usLF, (int)g_state.usLR, (int)g_state.usRF, (int)g_state.usRR,
      (int)g_state.usFront, (int)g_state.usBack,
      (int)g_state.usLeft, (int)g_state.usRight,
      (int)US_STOP_CM);
#endif

#if USE_LIDAR_HARDWARE
  Serial.println(F("[LiDAR] Boot sniff 450ms (mong thay 59 59 neu Luna gui dung)..."));
  uint32_t t0 = (uint32_t)millis();
  uint8_t raw1[40] = {0}, raw2[40] = {0};
  size_t n1 = 0, n2 = 0;
  while (((uint32_t)millis() - t0) < 450u) {
    while (Serial1.available() && n1 < sizeof(raw1)) raw1[n1++] = (uint8_t)Serial1.read();
    while (Serial2.available() && n2 < sizeof(raw2)) raw2[n2++] = (uint8_t)Serial2.read();
    delay(2);
  }
  auto dumpFn = [](const char *tag, const uint8_t *p, size_t n) {
    Serial.printf("  %s: %u byte — ", tag, (unsigned)n);
    for (size_t i = 0; i < n && i < 24; i++) Serial.printf("%02X ", p[i]);
    Serial.println();
  };
  dumpFn("Serial1 TRUOC (ESP RX=GPIO18)", raw1, n1);
  dumpFn("Serial2 SAU  (ESP RX=GPIO2)",  raw2, n2);
  if (n1 == 0 && n2 == 0) {
    Serial.println(F("  => Khong co byte UART. Kiem tra TX/RX, GND, 5V, chan MODE (khong keo GND)."));
  }

  Serial.println(F("[LiDAR] Poll 30 vong..."));
  for (int i = 0; i < 30; i++) {
    sensorsPollLidar();
    sensorsYieldMs(15);
  }
  sensorsPollUS();
  Serial.printf(
      "  LiDAR F:%d B:%d cm | US F:%d B:%d L:%d R:%d cm\n",
      (int)g_state.lidarFront, (int)g_state.lidarBack,
      (int)g_state.usFront, (int)g_state.usBack,
      (int)g_state.usLeft, (int)g_state.usRight);
  Serial.printf("  Bytes UART: Serial1=%lu Serial2=%lu\n",
                (unsigned long)g_lunaRxBytes1, (unsigned long)g_lunaRxBytes2);
#endif
}

#endif // SENSORS_H
