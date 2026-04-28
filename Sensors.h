/* =====================================================================
 *  Sensors.h — Cảm biến an toàn & định vị
 *    • 4x HC-SR04 (dùng chung TRIG — quét tuần tự bằng NewPing)
 *    • 2x TF-Luna LiDAR UART 9-byte frame
 *
 *  API:
 *    sensorsInit()     — Cấu hình TRIG chung + 2 Serial LiDAR
 *    sensorsPollUS()   — Quét 4 siêu âm (gọi từ task real-time)
 *    sensorsPollLidar()— Đọc 2 LiDAR (gọi liên tục từ task điều khiển)
 *    sensorsLogBootSample() — 1 lần lúc boot: in mẫu LiDAR/US ra Serial (debug phần cứng)
 * =====================================================================*/
#ifndef SENSORS_H
#define SENSORS_H

#include "Config.h"
#include <NewPing.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// 4 cảm biến siêu âm dùng cùng 1 chân TRIG (GPIO14)
static NewPing g_sonarF(US_TRIG, US_ECHO_F, US_PING_MAX_CM);
static NewPing g_sonarB(US_TRIG, US_ECHO_B, US_PING_MAX_CM);
static NewPing g_sonarL(US_TRIG, US_ECHO_L, US_PING_MAX_CM);
static NewPing g_sonarR(US_TRIG, US_ECHO_R, US_PING_MAX_CM);

/** Nghỉ giữa các ping (gọi từ task điều khiển / setup — dùng vTaskDelay, không busy-wait). */
inline void sensorsYieldMs(uint32_t ms) {
  const uint32_t m = ms ? ms : 1u;
  vTaskDelay(pdMS_TO_TICKS(m));
}

inline void sensorsInit() {
  // TRIG chung: mức nghỉ LOW trước khi NewPing chiếm dụng
  pinMode(US_TRIG, OUTPUT);
  digitalWrite(US_TRIG, LOW);

  // Bộ đệm RX lớn hơn — tránh mất byte LiDAR khi CPU bận (WiFi + task khác)
  Serial1.setRxBufferSize(1024);
  Serial2.setRxBufferSize(1024);
  // UART LiDAR trước & sau — TF-Luna mặc định 115200 8N1
  Serial1.begin(LIDAR_BAUD, SERIAL_8N1, LIDAR_F_RX, LIDAR_F_TX);
  Serial2.begin(LIDAR_BAUD, SERIAL_8N1, LIDAR_B_RX, LIDAR_B_TX);
  delay(80);
  while (Serial1.available()) (void)Serial1.read();
  while (Serial2.available()) (void)Serial2.read();
}

/**
 * Quét 4 siêu âm tuần tự vì dùng chung TRIG (GPIO14).
 * Ghi kết quả trực tiếp vào g_state. Giá trị 0 cm = ngoài tầm.
 * (Blocking ~3–4ms mỗi hướng với US_PING_MAX_CM=200)
 */
inline void sensorsPollUS() {
  g_state.usFront = g_sonarF.ping_cm();
  sensorsYieldMs(5);
  g_state.usBack = g_sonarB.ping_cm();
  sensorsYieldMs(5);
  g_state.usLeft = g_sonarL.ping_cm();
  sensorsYieldMs(5);
  g_state.usRight = g_sonarR.ping_cm();
  // 0 = không echo trong cửa sổ → coi như tầm tối đa ước tính (hành lang trống)
  if (g_state.usFront == 0) g_state.usFront = US_PING_MAX_CM;
  if (g_state.usBack  == 0) g_state.usBack  = US_PING_MAX_CM;
  if (g_state.usLeft  == 0) g_state.usLeft  = US_PING_MAX_CM;
  if (g_state.usRight == 0) g_state.usRight = US_PING_MAX_CM;
  g_state.usLastUpdateMs = millis();
}

/* ---------------------------------------------------------------
 *  TF-Luna frame 9 byte: 0x59 0x59  DistL DistH  StrL StrH  TempL TempH  CHK
 *  Checksum = lower byte của tổng 8 byte trước
 * --------------------------------------------------------------- */
inline bool readTfLuna(HardwareSerial &ser, int16_t &distCm) {
  static uint8_t bufF[9], bufB[9];
  static uint8_t idxF = 0, idxB = 0;
  // Mỗi Serial dùng buffer riêng — phân biệt qua ser.getBaseAddress? đơn giản là tham số tĩnh.
  uint8_t *buf = (&ser == &Serial1) ? bufF : bufB;
  uint8_t &idx = (&ser == &Serial1) ? idxF : idxB;

  bool gotFrame = false;
  while (ser.available()) {
    uint8_t b = ser.read();
    if (idx == 0) {
      if (b == 0x59) { buf[idx++] = b; }
    } else if (idx == 1) {
      if (b == 0x59) { buf[idx++] = b; } else { idx = 0; }
    } else {
      buf[idx++] = b;
      if (idx == 9) {
        uint16_t sum = 0;
        for (uint8_t i = 0; i < 8; i++) sum += buf[i];
        if ((sum & 0xFF) == buf[8]) {
          distCm = (int16_t)(buf[2] | (buf[3] << 8));  // cm
          gotFrame = true;
        }
        idx = 0;
      }
    }
  }
  return gotFrame;
}

inline void sensorsPollLidar() {
  int16_t d;
  if (readTfLuna(Serial1, d)) {
    if (d < 0) d = 0;
    if (d > (int16_t)LIDAR_MAX_CM) d = (int16_t)LIDAR_MAX_CM;
    g_state.lidarFront = d;
    g_state.lidarLastUpdateMs = millis();
  }
  if (readTfLuna(Serial2, d)) {
    if (d < 0) d = 0;
    if (d > (int16_t)LIDAR_MAX_CM) d = (int16_t)LIDAR_MAX_CM;
    g_state.lidarBack = d;
    g_state.lidarLastUpdateMs = millis();
  }
}

/**
 * Gọi từ setup() sau sensorsInit(). Đọc Serial 115200 để xem cảm biến có phản hồi không.
 */
inline void sensorsLogBootSample() {
  delay(200);
  Serial.println(F("[Sensors] Boot sample (30 vong LiDAR + 1 lan US)..."));
  for (int i = 0; i < 30; i++) {
    sensorsPollLidar();
    sensorsYieldMs(15);
  }
  sensorsPollUS();
  Serial.printf(
      "  LiDAR F:%d B:%d cm | US F:%d B:%d L:%d R:%d cm\n",
      (int)g_state.lidarFront, (int)g_state.lidarBack, (int)g_state.usFront,
      (int)g_state.usBack, (int)g_state.usLeft, (int)g_state.usRight);
  Serial.printf(
      "  UART dinh du lieu? Serial1.rx~%d Serial2.rx~%d byte (0 = thuong la sai day/baud/chua cap Luna)\n",
      Serial1.available(), Serial2.available());
}

#endif // SENSORS_H
