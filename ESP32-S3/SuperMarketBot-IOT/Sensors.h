/* =====================================================================
 *  Sensors.h — Cảm biến an toàn & định vị
 *    • 4× HC-SR04 (4 góc) khi USE_HC_SR04_HARDWARE=1
 *    • 2× TF-Luna khi USE_LIDAR_HARDWARE=1
 *
 *  Khi chỉ LiDAR: sensorsPollUS() shadow từ Luna trước/sau
 *  (hai bên = “xa” LIDAR_MAX_CM — không có mắt ngang).
 *
 *  API:
 *    sensorsInit()     — UART LiDAR (+ TRIG/Echo SR04 nếu bật)
 *    sensorsPollUS()   — SR04 hoặc shadow từ LiDAR
 *    sensorsPollLidar()— Đọc 2 LiDAR
 *    sensorsLogBootSample() — boot debug
 * =====================================================================*/
#ifndef SENSORS_H
#define SENSORS_H

#include "Config.h"
#if USE_HC_SR04_HARDWARE
#include <NewPing.h>
#endif
#include "SensorLayout.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

volatile uint32_t g_lunaRxBytes1 = 0;
volatile uint32_t g_lunaRxBytes2 = 0;
volatile uint32_t g_luna1LastOkMs = 0;
volatile uint32_t g_luna2LastOkMs = 0;
volatile uint32_t g_usPhyLastEchoMs[4] = {0, 0, 0, 0};
int g_batPct = -1; // Biến toàn cục lưu phần trăm pin cho MQTT và AutoDock

#if USE_HC_SR04_HARDWARE

static int16_t s_usFiltered[4] = {
  (int16_t)US_PING_MAX_CM, (int16_t)US_PING_MAX_CM,
  (int16_t)US_PING_MAX_CM, (int16_t)US_PING_MAX_CM
};
/** Lịch sử 3 mẫu cho Median Filter mỗi cảm biến */
static int16_t s_usHistory[4][3] = {
  {(int16_t)US_PING_MAX_CM, (int16_t)US_PING_MAX_CM, (int16_t)US_PING_MAX_CM},
  {(int16_t)US_PING_MAX_CM, (int16_t)US_PING_MAX_CM, (int16_t)US_PING_MAX_CM},
  {(int16_t)US_PING_MAX_CM, (int16_t)US_PING_MAX_CM, (int16_t)US_PING_MAX_CM},
  {(int16_t)US_PING_MAX_CM, (int16_t)US_PING_MAX_CM, (int16_t)US_PING_MAX_CM}
};
/** Bộ lọc thông thấp EMA cho cảm biến siêu âm */
static float s_usEma[4] = {
  (float)US_PING_MAX_CM, (float)US_PING_MAX_CM,
  (float)US_PING_MAX_CM, (float)US_PING_MAX_CM
};

static inline int16_t getMedian3(int16_t a, int16_t b, int16_t c) {
  if ((a <= b && b <= c) || (c <= b && b <= a)) return b;
  if ((b <= a && a <= c) || (c <= a && a <= b)) return a;
  return c;
}

/** Lọc nhiễu SR04 bằng kết hợp Median Filter và EMA (Exponential Moving Average) */
static inline int16_t usFilterSample(uint8_t idx, int16_t raw) {
  if (raw <= 0) raw = (int16_t)US_PING_MAX_CM;
  if (raw > (int16_t)US_PING_MAX_CM) raw = (int16_t)US_PING_MAX_CM;
  if (raw < (int16_t)US_MIN_VALID_CM) raw = (int16_t)US_PING_MAX_CM;

  // Nếu bộ lọc vừa bị reset (có giá trị -1)
  if (s_usHistory[idx][0] == -1) {
    s_usHistory[idx][0] = raw;
    s_usHistory[idx][1] = raw;
    s_usHistory[idx][2] = raw;
    s_usEma[idx] = (float)raw;
  } else {
    // Dịch chuyển lịch sử
    s_usHistory[idx][0] = s_usHistory[idx][1];
    s_usHistory[idx][1] = s_usHistory[idx][2];
    s_usHistory[idx][2] = raw;
  }

  int16_t median = getMedian3(s_usHistory[idx][0], s_usHistory[idx][1], s_usHistory[idx][2]);
  
  // EMA Filter: phản hồi nhanh khi tiến lại gần vật cản, mịn khi ở xa
  float rawF = (float)median;
  if (rawF < s_usEma[idx]) {
    s_usEma[idx] = 0.7f * rawF + 0.3f * s_usEma[idx]; // Gặp vật cản: bám nhanh (tránh trễ)
  } else {
    s_usEma[idx] = 0.2f * rawF + 0.8f * s_usEma[idx]; // Rút xa: hồi phục mịn màng
  }

  int16_t filtered = (int16_t)(s_usEma[idx] + 0.5f);
  s_usFiltered[idx] = filtered;
  return filtered;
}

inline void usFilterReset() {
  for (int i = 0; i < 4; i++) {
    s_usHistory[i][0] = -1;
    s_usHistory[i][1] = -1;
    s_usHistory[i][2] = -1;
    s_usEma[i] = (float)US_PING_MAX_CM;
  }
}
#else
inline void usFilterReset() {}
#endif

/** Nghỉ giữa các ping (gọi từ task điều khiển / setup — dùng vTaskDelay, không busy-wait). */
inline void sensorsYieldMs(uint32_t ms) {
  const uint32_t m = ms ? ms : 1u;
  vTaskDelay(pdMS_TO_TICKS(m));
}

/** Gửi 1 frame lệnh Benewake (Head 0x5A, Len, ID, payload..., checksum = tổng byte trước đó & 0xFF). */
inline void tflunaSendFrame(HardwareSerial &ser, const uint8_t *frame, size_t len) {
  if (len < 4) return;
  ser.write(frame, len);
  ser.flush();
  delay(25);
  while (ser.available()) (void)ser.read();
}

#if TFLUNA_SEND_INIT_CMD
/**
 * Đưa TF-Luna về UART stream 9 byte (cm), bật output, đặt FPS, lưu flash module.
 * Nếu chân Mode (thường chân 5) đang kéo GND → module ở I2C, UART sẽ không có 0x59 0x59.
 */
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

inline void sensorsInit() {
#if USE_HC_SR04_HARDWARE
  pinMode(US_TRIG, OUTPUT);
  digitalWrite(US_TRIG, LOW);
  pinMode(US_ECHO_LF, INPUT_PULLDOWN);
  pinMode(US_ECHO_RL, INPUT_PULLDOWN);
  pinMode(US_ECHO_RF, INPUT_PULLDOWN);
  pinMode(US_ECHO_RR, INPUT_PULLDOWN);
  Serial.println(F("[US] HC-SR04 x4 (LF/RL/RF/RR) — stop <30cm, OA tu 42cm."));
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
  Serial.println(F("[LiDAR] TF-Luna UART init."));
#endif
#else
  g_state.lidarFront = (int16_t)LIDAR_MAX_CM;
  g_state.lidarBack  = (int16_t)LIDAR_MAX_CM;
#endif
}

/** Gán 4 khoảng cách vật lý (F,B,L,R) → góc xe + usFront/Back/Left/Right. */
inline void sensorsCommitPhyToState(const int16_t phy[4]) {
  int16_t usSlot[4];
  for (int s = 0; s < 4; s++) {
    uint8_t p = g_mapUsSlot[s];
    if (p > 3) p = (uint8_t)s;
    usSlot[s] = phy[p];
  }
  // Gán 4 cổng vật lý trực tiếp cho 4 hướng logic: F (0), B (1), L (2), R (3)
  g_state.usLF = usSlot[0]; // Port 0 đại diện cho Front
  g_state.usLR = usSlot[1]; // Port 1 đại diện cho Back
  g_state.usRF = usSlot[2]; // Port 2 đại diện cho Left
  g_state.usRR = usSlot[3]; // Port 3 đại diện cho Right

  g_state.usFront = g_state.usLF;
  g_state.usBack  = g_state.usLR;
  g_state.usLeft  = g_state.usRF;
  g_state.usRight = g_state.usRR;
  g_state.usLastUpdateMs = millis();
}

/**
 * SR04 (nếu bật) hoặc đồng bộ từ LiDAR — gọi sau sensorsPollLidar() khi không dùng SR04.
 */
inline int16_t readCustomSonar(uint8_t triggerPin, uint8_t echoPin) {
  // 1. Đảm bảo chân Trig ở mức LOW trước
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  
  // 2. Kích hoạt phát sóng bằng cách kéo Trig lên HIGH trong 10us
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
  
  // 3. Đo thời gian phản hồi HIGH trên chân Echo
  // 15000us tương đương khoảng ~250cm (an toàn và phản hồi nhanh)
  uint32_t duration = pulseIn(echoPin, HIGH, 15000UL);
  if (duration == 0) return 0;
  
  // Khoảng cách (cm) = thời gian / 58
  return (int16_t)(duration / 58);
}

inline void sensorsPollUS() {
#if USE_HC_SR04_HARDWARE
  static int16_t phy[4] = {
    (int16_t)US_PING_MAX_CM, (int16_t)US_PING_MAX_CM,
    (int16_t)US_PING_MAX_CM, (int16_t)US_PING_MAX_CM
  };
  static uint8_t currentSensorIdx = 0;
  int16_t r = 0;
  
  switch (currentSensorIdx) {
    case US_PHY_F: // LF (Trái trước)
      r = readCustomSonar(US_TRIG, US_ECHO_LF);
      phy[US_PHY_F] = usFilterSample(US_PHY_F, r);
      g_usPhyLastEchoMs[US_PHY_F] = millis();
      break;
    case US_PHY_B: // RL (Trái sau)
      r = readCustomSonar(US_TRIG, US_ECHO_RL);
      phy[US_PHY_B] = usFilterSample(US_PHY_B, r);
      g_usPhyLastEchoMs[US_PHY_B] = millis();
      break;
    case US_PHY_L: // RF (Phải trước)
      r = readCustomSonar(US_TRIG, US_ECHO_RF);
      phy[US_PHY_L] = usFilterSample(US_PHY_L, r);
      g_usPhyLastEchoMs[US_PHY_L] = millis();
      break;
    case US_PHY_R: // RR (Phải sau)
      r = readCustomSonar(US_TRIG, US_ECHO_RR);
      phy[US_PHY_R] = usFilterSample(US_PHY_R, r);
      g_usPhyLastEchoMs[US_PHY_R] = millis();
      break;
  }

  sensorsCommitPhyToState(phy);
  currentSensorIdx = (currentSensorIdx + 1) % 4;
#else
  int16_t phy[4];
  phy[US_PHY_F] = g_state.lidarFront;
  phy[US_PHY_B] = g_state.lidarBack;
  phy[US_PHY_L] = (int16_t)LIDAR_MAX_CM;
  phy[US_PHY_R] = (int16_t)LIDAR_MAX_CM;
  sensorsCommitPhyToState(phy);
  g_state.usLeft = (int16_t)LIDAR_MAX_CM;
  g_state.usRight = (int16_t)LIDAR_MAX_CM;
  {
    uint32_t tF = (g_lidarFrontUart == 0u) ? g_luna1LastOkMs : g_luna2LastOkMs;
    uint32_t tB = (g_lidarFrontUart == 0u) ? g_luna2LastOkMs : g_luna1LastOkMs;
    g_usPhyLastEchoMs[US_PHY_F] = tF;
    g_usPhyLastEchoMs[US_PHY_B] = tB;
    g_usPhyLastEchoMs[US_PHY_L] = 0;
    g_usPhyLastEchoMs[US_PHY_R] = 0;
  }
#endif
}

/* ---------------------------------------------------------------
 *  TF-Luna frame 9 byte: 0x59 0x59  DistL DistH  StrL StrH  TempL TempH  CHK
 *  Checksum = lower byte của tổng 8 byte trước
 * --------------------------------------------------------------- */
inline bool readTfLunaStream(HardwareSerial &ser, uint8_t *buf, uint8_t &idx,
                           volatile uint32_t &rxCount, int16_t &distCm) {
  bool gotFrame = false;
  while (ser.available()) {
    uint8_t b = (uint8_t)ser.read();
    rxCount++;
    if (idx == 0) {
      if (b == 0x59u) {
        buf[idx++] = b;
      }
    } else if (idx == 1) {
      if (b == 0x59u) {
        buf[idx++] = b;
      } else {
        idx = 0;
      }
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

inline void sensorsPollLidar() {
#if !USE_LIDAR_HARDWARE
  return;
#endif
  static uint8_t buf1[9], buf2[9];
  static uint8_t idx1 = 0, idx2 = 0;
  static int16_t rawD1 = (int16_t)LIDAR_MAX_CM;
  static int16_t rawD2 = (int16_t)LIDAR_MAX_CM;
  static float emaL1 = (float)LIDAR_MAX_CM;
  static float emaL2 = (float)LIDAR_MAX_CM;
  int16_t d;
  if (readTfLunaStream(Serial1, buf1, idx1, g_lunaRxBytes1, d)) {
    if (d < 0) d = 0;
    if (d > (int16_t)LIDAR_MAX_CM) d = (int16_t)LIDAR_MAX_CM;
    
    // EMA Filter cho LiDAR 1 (TF-Luna 100Hz)
    float rawF = (float)d;
    if (rawF < emaL1) {
      emaL1 = 0.8f * rawF + 0.2f * emaL1; // Vật cản tiến lại gần: bám nhanh bảo vệ an toàn
    } else {
      emaL1 = 0.15f * rawF + 0.85f * emaL1; // Rút xa: mượt mà chống nhảy số
    }
    rawD1 = (int16_t)(emaL1 + 0.5f);
    g_luna1LastOkMs = (uint32_t)millis();
    g_state.lidarLastUpdateMs = millis();
  }
  if (readTfLunaStream(Serial2, buf2, idx2, g_lunaRxBytes2, d)) {
    if (d < 0) d = 0;
    if (d > (int16_t)LIDAR_MAX_CM) d = (int16_t)LIDAR_MAX_CM;
    
    // EMA Filter cho LiDAR 2
    float rawF = (float)d;
    if (rawF < emaL2) {
      emaL2 = 0.8f * rawF + 0.2f * emaL2;
    } else {
      emaL2 = 0.15f * rawF + 0.85f * emaL2;
    }
    rawD2 = (int16_t)(emaL2 + 0.5f);
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

/**
 * Gọi từ setup() sau sensorsInit(). Đọc Serial 115200 để xem cảm biến có phản hồi không.
 */
inline void sensorsLogBootSample() {
  delay(150);
#if USE_HC_SR04_HARDWARE
  Serial.println(F("[US] Boot ping 4 goc (24 vong)..."));
  for (int i = 0; i < 24; i++) {
    sensorsPollUS();
    sensorsYieldMs(30);
  }
  Serial.printf(
      "  US LF:%d RL:%d RF:%d RR:%d | F:%d B:%d L:%d R:%d cm (stop<%d)\n",
      (int)g_state.usLF, (int)g_state.usLR, (int)g_state.usRF, (int)g_state.usRR,
      (int)g_state.usFront, (int)g_state.usBack, (int)g_state.usLeft, (int)g_state.usRight,
      (int)US_STOP_CM);
  return;
#endif
  Serial.println(F("[LiDAR] Raw sniff 450ms (mong thay 59 59 neu Luna gui dung)..."));
  uint32_t t0 = (uint32_t)millis();
  uint8_t raw1[40], raw2[40];
  size_t n1 = 0, n2 = 0;
  while (((uint32_t)millis() - t0) < 450u) {
    while (Serial1.available() && n1 < sizeof(raw1)) raw1[n1++] = (uint8_t)Serial1.read();
    while (Serial2.available() && n2 < sizeof(raw2)) raw2[n2++] = (uint8_t)Serial2.read();
    delay(2);
  }
  auto dump = [](const char *tag, const uint8_t *p, size_t n) {
    Serial.printf("  %s: %u byte — ", tag, (unsigned)n);
    for (size_t i = 0; i < n && i < 24; i++) Serial.printf("%02X ", p[i]);
    Serial.println();
  };
  dump("Serial1 TRUOC (ESP RX=GPIO18, Luna TX noi vao day)", raw1, n1);
  dump("Serial2 SAU (ESP RX=GPIO2, Luna TX noi vao day)", raw2, n2);
  if (n1 == 0 && n2 == 0) {
    Serial.println(
        F("  => Khong co byte UART. Kiem tra:"));
    Serial.println(
        F("     - Noi Luna TX -> ESP RX (truoc: RX=GPIO18, sau: RX=GPIO2); RX Luna <- ESP TX."));
    Serial.println(
        F("     - GND chung; nguon 5V on dinh."));
    Serial.println(
        F("     - TF-Luna: chan MODE (thuong chan 5) KHONG duoc keo GND → GND = che do I2C (UART im)."));
    Serial.println(
        F("     - Thu doi cap TX/RX; dam bao baud 115200 (Config LIDAR_BAUD)."));
  } else if (n1 > 0 || n2 > 0) {
    Serial.println(F("  (Neu khong thay 59 59 trong dump: sai baud, output tat, hoac dang che do I2C.)"));
  }

  Serial.println(F("[Sensors] Boot doc (30 vong LiDAR + dong bo khoang cach)..."));
  for (int i = 0; i < 30; i++) {
    sensorsPollLidar();
    sensorsYieldMs(15);
  }
  sensorsPollUS();
  Serial.printf(
      "  LiDAR F:%d B:%d cm | Shadow F:%d B:%d L:%d R:%d cm (L/R=max neu khong SR04)\n",
      (int)g_state.lidarFront, (int)g_state.lidarBack, (int)g_state.usFront,
      (int)g_state.usBack, (int)g_state.usLeft, (int)g_state.usRight);
  Serial.printf(
      "  Bytes tong: Serial1=%lu Serial2=%lu (tang lien tuc khi co du lieu)\n",
      (unsigned long)g_lunaRxBytes1, (unsigned long)g_lunaRxBytes2);
  Serial.printf(
      "  UART queue con lai: Serial1=%d Serial2=%d\n", Serial1.available(),
      Serial2.available());
}

#endif // SENSORS_H
