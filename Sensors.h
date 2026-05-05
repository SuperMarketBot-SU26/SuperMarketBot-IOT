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
#include "SensorLayout.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

volatile uint32_t g_lunaRxBytes1 = 0;
volatile uint32_t g_lunaRxBytes2 = 0;
volatile uint32_t g_luna1LastOkMs = 0;
volatile uint32_t g_luna2LastOkMs = 0;
volatile uint32_t g_usPhyLastEchoMs[4] = {0, 0, 0, 0};

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
  // TRIG chung: mức nghỉ LOW trước khi NewPing chiếm dụng
  pinMode(US_TRIG, OUTPUT);
  digitalWrite(US_TRIG, LOW);

  // Bộ đệm RX lớn hơn — tránh mất byte LiDAR khi CPU bận (WiFi + task khác)
  Serial1.setRxBufferSize(1024);
  Serial2.setRxBufferSize(1024);
  // UART LiDAR trước & sau — TF-Luna mặc định 115200 8N1
  Serial1.begin(LIDAR_BAUD, SERIAL_8N1, LIDAR_F_RX, LIDAR_F_TX);
  Serial2.begin(LIDAR_BAUD, SERIAL_8N1, LIDAR_B_RX, LIDAR_B_TX);
  delay(120);
  while (Serial1.available()) (void)Serial1.read();
  while (Serial2.available()) (void)Serial2.read();

#if TFLUNA_SEND_INIT_CMD
  tflunaUartApplyDefaults(Serial1);
  tflunaUartApplyDefaults(Serial2);
  Serial.println(F("[LiDAR] Da gui lenh TF-Luna: bat output UART + khung 9 byte (cm) + save."));
#endif
}

/**
 * Quét 4 siêu âm tuần tự vì dùng chung TRIG (GPIO14).
 * Ghi kết quả trực tiếp vào g_state. Giá trị 0 cm = ngoài tầm.
 * (Blocking ~3–4ms mỗi hướng với US_PING_MAX_CM=200)
 */
inline void sensorsPollUS() {
  uint32_t tnow = (uint32_t)millis();
  int16_t phy[4];
  phy[0] = (int16_t)g_sonarF.ping_cm();
  sensorsYieldMs(5);
  phy[1] = (int16_t)g_sonarB.ping_cm();
  sensorsYieldMs(5);
  phy[2] = (int16_t)g_sonarL.ping_cm();
  sensorsYieldMs(5);
  phy[3] = (int16_t)g_sonarR.ping_cm();
  for (int i = 0; i < 4; i++) {
    if (phy[i] > 0) g_usPhyLastEchoMs[i] = tnow;
    if (phy[i] == 0) phy[i] = US_PING_MAX_CM;
  }
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
  g_state.usLastUpdateMs = millis();
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
  static uint8_t buf1[9], buf2[9];
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

/**
 * Gọi từ setup() sau sensorsInit(). Đọc Serial 115200 để xem cảm biến có phản hồi không.
 */
inline void sensorsLogBootSample() {
  delay(150);
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

  Serial.println(F("[Sensors] Boot doc (30 vong parse + 1 lan US)..."));
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
      "  Bytes tong: Serial1=%lu Serial2=%lu (tang lien tuc khi co du lieu)\n",
      (unsigned long)g_lunaRxBytes1, (unsigned long)g_lunaRxBytes2);
  Serial.printf(
      "  UART queue con lai: Serial1=%d Serial2=%d\n", Serial1.available(),
      Serial2.available());
}

#endif // SENSORS_H
