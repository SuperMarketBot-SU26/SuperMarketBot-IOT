/* =====================================================================
 *  Sensors.h — Cảm biến an toàn & định vị
 *    • 4× HC-SR04 (4 góc) khi USE_HC_SR04_HARDWARE=1
 *    • YDLIDAR X3 360° khi USE_YDLIDAR_X3=1 (UART Serial1)
 *
 *  TF-Luna đã bỏ hoàn toàn (2026-07-30).
 *
 *  API:
 *    sensorsInit()     — Khởi tạo SR04 TRIG/ECHO + YDLIDAR X3 UART
 *    sensorsPollUS()  — SR04 hardware polling (1 sensor mỗi tick)
 *    sensorsPollLidar()— YDLIDAR X3 polling (gọi trong taskX3 riêng)
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

volatile uint32_t g_usPhyLastEchoMs[4] = {0, 0, 0, 0};
volatile uint32_t g_encPhyLastPulseMs[4] = {0, 0, 0, 0};
int g_batPct = -1;

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

  if (s_usHistory[idx][0] == -1) {
    s_usHistory[idx][0] = raw;
    s_usHistory[idx][1] = raw;
    s_usHistory[idx][2] = raw;
    s_usEma[idx] = (float)raw;
  } else {
    s_usHistory[idx][0] = s_usHistory[idx][1];
    s_usHistory[idx][1] = s_usHistory[idx][2];
    s_usHistory[idx][2] = raw;
  }

  int16_t median = getMedian3(s_usHistory[idx][0], s_usHistory[idx][1], s_usHistory[idx][2]);

  float rawF = (float)median;
  if (rawF < s_usEma[idx]) {
    s_usEma[idx] = 0.7f * rawF + 0.3f * s_usEma[idx];
  } else {
    s_usEma[idx] = 0.2f * rawF + 0.8f * s_usEma[idx];
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

inline void sensorsInit() {
#if USE_HC_SR04_HARDWARE
  pinMode(US_TRIG, OUTPUT);
  digitalWrite(US_TRIG, LOW);
  pinMode(US_ECHO_LF, INPUT_PULLDOWN);
  pinMode(US_ECHO_RL, INPUT_PULLDOWN);
  pinMode(US_ECHO_RF, INPUT_PULLDOWN);
  pinMode(US_ECHO_RR, INPUT_PULLDOWN);
  Serial.println(F("[US] HC-SR04 x4 (LF/RL/RF/RR) — TRIG=GPIO14, ECHO=10/11/12/13."));
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
  g_state.usLF = usSlot[0];
  g_state.usLR = usSlot[1];
  g_state.usRF = usSlot[2];
  g_state.usRR = usSlot[3];

  g_state.usFront = g_state.usLF;
  g_state.usBack  = g_state.usLR;
  g_state.usLeft  = g_state.usRF;
  g_state.usRight = g_state.usRR;
  g_state.usLastUpdateMs = millis();
}

/**
 * Đọc 1 cảm biến HC-SR04 (trigger chung, đọc lần lượt từng ECHO).
 */
inline int16_t readCustomSonar(uint8_t triggerPin, uint8_t echoPin) {
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);

  uint32_t duration = pulseIn(echoPin, HIGH, 15000UL);
  if (duration == 0) return 0;
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
    case US_PHY_F:
      r = readCustomSonar(US_TRIG, US_ECHO_LF);
      phy[US_PHY_F] = usFilterSample(US_PHY_F, r);
      g_usPhyLastEchoMs[US_PHY_F] = millis();
      break;
    case US_PHY_B:
      r = readCustomSonar(US_TRIG, US_ECHO_RL);
      phy[US_PHY_B] = usFilterSample(US_PHY_B, r);
      g_usPhyLastEchoMs[US_PHY_B] = millis();
      break;
    case US_PHY_L:
      r = readCustomSonar(US_TRIG, US_ECHO_RF);
      phy[US_PHY_L] = usFilterSample(US_PHY_L, r);
      g_usPhyLastEchoMs[US_PHY_L] = millis();
      break;
    case US_PHY_R:
      r = readCustomSonar(US_TRIG, US_ECHO_RR);
      phy[US_PHY_R] = usFilterSample(US_PHY_R, r);
      g_usPhyLastEchoMs[US_PHY_R] = millis();
      break;
  }

  sensorsCommitPhyToState(phy);
  currentSensorIdx = (currentSensorIdx + 1) % 4;
#endif
}

/**
 * sensorsPollLidar() — YDLIDAR X3 đã chuyển sang taskX3 riêng trong YdlidarX3.h.
 * Hàm này giữ lại để tương thích — không làm gì.
 */
inline void sensorsPollLidar() {
  // YDLIDAR X3 scan done in taskX3 (YdlidarX3.h)
}

inline void sensorsLogBootSample() {
#if USE_HC_SR04_HARDWARE
  delay(150);
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
#endif
}

#endif // SENSORS_H
