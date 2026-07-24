/* =====================================================================
 *  CtrlJson.h — Lệnh điều khiển JSON (WebSocket dashboard)
 * =====================================================================*/
#ifndef CTRLJSON_H
#define CTRLJSON_H

#include "Config.h"
// Motors.h bị loại khỏi đây vì kéo theo MotorLayout.h → WebSocketsServer → WiFi
// botStop() được extern định nghĩa trong Motors.h (đã include ở .ino)
extern void botStop();
#include "WaypointNav.h"
#include "LineDecoder.h"   // cần extern g_lineSpeedPct (slider mode LINE)
#include <ArduinoJson.h>
#include <Preferences.h>
#include <freertos/FreeRTOS.h>

// Fallback: declare extern g_lineSpeedPct trực tiếp nếu LineDecoder.h
// không include kịp (compile order issue).
extern uint8_t g_lineSpeedPct;
#include <freertos/semphr.h>
#include <cstring>

extern RobotState g_state;
extern SemaphoreHandle_t g_stateMutex;
extern Preferences g_prefs;

/** Dừng motor + về lái tay (gọi khi boot / E-Stop / đổi mode Manual). */
inline void robotForceManualStop() {
  g_state.mode = MODE_MANUAL;
  g_state.cmdX = 0;
  g_state.cmdY = 0;
  g_state.cmdStrafe = 0;
  g_state.estop = false;
  botStop();
  wpNavCancel();
}

inline void robotApplyControlJson(JsonDocument &doc) {
  const char *t = doc["t"];
  if (!t) return;

  if (strcmp(t, "joy") == 0) {
    /* Bọc mutex: đọc từ Core 0 (WebSocket), ghi vào g_state.
     * Core 1 (Control) sẽ đọc giá trị đồng nhất của cmdX/Y/Strafe. */
    if (g_stateMutex != NULL) xSemaphoreTake(g_stateMutex, portMAX_DELAY);
    g_state.cmdX = (int16_t)constrain((int)doc["x"].as<int>(), -100, 100);
    g_state.cmdY = (int16_t)constrain((int)doc["y"].as<int>(), -100, 100);
    g_state.cmdStrafe = (int16_t)constrain((int)doc["s"].as<int>(), -100, 100);
    if (g_stateMutex != NULL) xSemaphoreGive(g_stateMutex);

    // In log debug mỗi 500ms khi lái
    static uint32_t lastJoyLog = 0;
    if (millis() - lastJoyLog > 500u) {
      lastJoyLog = millis();
      Serial.printf("[WS-Joy] X:%d, Y:%d, Strafe:%d\n", g_state.cmdX, g_state.cmdY, g_state.cmdStrafe);
    }
  } else if (strcmp(t, "spd") == 0) {
    uint16_t pct = doc["v"].as<uint16_t>();
    if (pct > 100) pct = 100;
    g_state.baseSpeed = (uint16_t)((uint32_t)pct * PWM_MAX / 100);
    g_prefs.begin(NVS_NAMESPACE, false);
    g_prefs.putUInt("baseSpeed", g_state.baseSpeed);
    g_prefs.end();
  } else if (strcmp(t, "spdAuto") == 0) {
    uint16_t pct = doc["v"].as<uint16_t>();
    if (pct > 100) pct = 100;
    g_state.autoBaseSpeed = (uint16_t)((uint32_t)pct * PWM_MAX / 100);
    g_prefs.begin(NVS_NAMESPACE, false);
    g_prefs.putUInt("autoBaseSpeed", g_state.autoBaseSpeed);
    g_prefs.end();
  } else if (strcmp(t, "spdSwerve") == 0) {
    uint16_t pct = doc["v"].as<uint16_t>();
    if (pct > 100) pct = 100;
    g_state.swerveBaseSpeed = (uint16_t)((uint32_t)pct * PWM_MAX / 100);
    g_prefs.begin(NVS_NAMESPACE, false);
    g_prefs.putUInt("swerveSpeed", g_state.swerveBaseSpeed);
    g_prefs.end();
  } else if (strcmp(t, "spdLine") == 0) {
    uint16_t pct = doc["v"].as<uint16_t>();
    if (pct > 100) pct = 100;
    if (pct < 15) pct = 15;   // tối thiểu 15% cho motor torque
    g_lineSpeedPct = (uint8_t)pct;
    g_prefs.begin(NVS_NAMESPACE, false);
    g_prefs.putUChar("lineSpeedPct", g_lineSpeedPct);
    g_prefs.end();
    Serial.printf("[LINE-SPD] Slider -> %u%%\n", (unsigned)g_lineSpeedPct);
  } else if (strcmp(t, "spdRotate") == 0) {
    uint16_t pct = doc["v"].as<uint16_t>();
    if (pct > 100) pct = 100;
    g_state.rotateBaseSpeed = (uint16_t)((uint32_t)pct * PWM_MAX / 100);
    g_prefs.begin(NVS_NAMESPACE, false);
    g_prefs.putUInt("rotateSpeed", g_state.rotateBaseSpeed);
    g_prefs.end();
  } else if (strcmp(t, "yawScale") == 0) {
    uint16_t pct = doc["v"].as<uint16_t>();
    if (pct < 50) pct = 50;
    if (pct > 150) pct = 150;
    g_state.imuYawScale = (float)pct / 100.0f;
    g_prefs.begin(NVS_NAMESPACE, false);
    g_prefs.putUInt("yawScale", pct);
    g_prefs.end();
  } else if (strcmp(t, "mode") == 0) {
    uint8_t m = doc["m"].as<uint8_t>();
    Serial.printf("[WS-Mode] Yeu cau chuyen sang Mode: %d\n", m);
    if (m > MODE_LINE) m = MODE_MANUAL;
    if (m == MODE_MANUAL) {
      robotForceManualStop();
    } else {
      g_state.mode = (RobotMode)m;
      g_state.cmdX = 0;
      g_state.cmdY = 0;
      g_state.cmdStrafe = 0;
      botStop();
      if (m == MODE_WAYPOINT) {
        if (s_wpCount == 0) {
          Waypoint pts[1] = {{1.0f, 0.0f, 1}};
          wpNavSetRoute(pts, 1);
        }
        wpNavStart();
      }
    }
  } else if (strcmp(t, "estop") == 0) {
    Serial.println(F("[WS-EStop] KICH HOAT ESTOP!"));
    g_state.estop = true;
    botStop();
    wpNavCancel();
  } else if (strcmp(t, "odomReset") == 0) {
    extern void odomResetDistance();
    odomResetDistance();
  } else if (strcmp(t, "slam_pose") == 0) {
    // Nhận tọa độ SLAM hiệu chỉnh từ WebManager (PC chạy Scan-to-Scan Matching)
    // Format: { t: "slam_pose", x: float_m, y: float_m, h: float_rad }
    float sx = doc["x"] | 0.f;
    float sy = doc["y"] | 0.f;
    float sh = doc["h"] | 0.f;
    locSetSlamPose(sx, sy, sh);
  } else if (strcmp(t, "test_motor") == 0) {
    const char *payloadStr = doc["payload"] | "";
    int slot = -1;
    int speedPct = 0;
    if (sscanf(payloadStr, "%d_%d", &slot, &speedPct) == 2) {
      robotForceManualStop(); // Chuyển về lái tay và dừng các động cơ khác
      if (slot >= 0 && slot < 4) {
        int32_t speedVal = (int32_t)PWM_MAX * speedPct / 100;
        int32_t sp[4] = {0, 0, 0, 0};
        sp[slot] = speedVal;
        extern void motorApplyLayout(const int32_t speedBySlot[4]);
        motorApplyLayout(sp);
        Serial.printf("[WS-TestMotor] Slot %d, Speed %d%%\n", slot, speedPct);
      }
    }
  } else if (strcmp(t, "navigate") == 0) {
    const char *payloadStr = doc["payload"] | "";
    Serial.printf("[WS-Navigate] Nhan lo trinh tu WebSocket! Payload: %s\n", payloadStr);
    extern bool wpNavParseAndStart(const char *jsonPayload);
    if (!wpNavParseAndStart(payloadStr)) {
      Serial.println(F("[WS-Navigate ERROR] Loi parse hoac start lo trinh tu WebSocket!"));
    }
  } else if (strcmp(t, "wheelMode") == 0) {
    // Đã bỏ — hệ thống cố định differential drive (bánh thường). Bỏ qua lệnh.
    Serial.println(F("[WS-WheelMode] Bo qua — chi dung differential drive"));
  } else if (strcmp(t, "motorInv") == 0) {
    // Đảo chiều bánh: payload = "slot_invert" (VD: "0_1" = đảo bánh 0)
    const char *payloadStr = doc["payload"] | "";
    int slot = -1;
    int invert = -1;
    if (sscanf(payloadStr, "%d_%d", &slot, &invert) == 2) {
      if (slot >= 0 && slot < 4 && (invert == 0 || invert == 1)) {
        extern void motorInvertSlot(uint8_t slot, uint8_t invert);
        motorInvertSlot((uint8_t)slot, (uint8_t)invert);
        // Lưu vào NVS
        extern bool motorLayoutSaveCurrent(Preferences &prefs);
        motorLayoutSaveCurrent(g_prefs);
        Serial.printf("[WS-MotorInv] Slot %d -> %s\n", slot, invert ? "DAO CHIEU" : "BINH THUONG");
      }
    }
  } else if (strcmp(t, "motorInvToggle") == 0) {
    // Toggle đảo chiều 1 bánh: payload = "slot"
    const char *payloadStr = doc["payload"] | "";
    int slot = -1;
    if (sscanf(payloadStr, "%d", &slot) == 1) {
      if (slot >= 0 && slot < 4) {
        extern uint8_t motorLayoutToggleInvert(uint8_t slot);
        uint8_t newVal = motorLayoutToggleInvert((uint8_t)slot);
        // Lưu vào NVS
        extern bool motorLayoutSaveCurrent(Preferences &prefs);
        motorLayoutSaveCurrent(g_prefs);
        Serial.printf("[WS-MotorInvToggle] Slot %d -> %s\n", slot, newVal ? "DAO CHIEU" : "BINH THUONG");
      }
    }
  } else if (strcmp(t, "motorMap") == 0) {
    // Cập nhật motor layout: payload = "[0,1,2,3]" (map) + "[0,0,0,0]" (inv)
    // Xử lý trong motorLayoutApplyJson
    extern bool motorLayoutApplyJson(JsonDocument &doc, Preferences &prefs);
    if (motorLayoutApplyJson((JsonDocument&)doc, g_prefs)) {
      Serial.println("[WS-MotorMap] Cap nhat thanh cong!");
    } else {
      Serial.println("[WS-MotorMap] Loi cap nhat!");
    }
  } else if (strcmp(t, "motorTestAll") == 0) {
    // Test từng bánh 1 để kiểm tra đảo chiều
    // payload = "1" hoặc "-1" (chiều quay)
    const char *payloadStr = doc["payload"] | "";
    int direction = 1;
    sscanf(payloadStr, "%d", &direction);
    int32_t speedVal = (int32_t)PWM_MAX * direction;
    int32_t sp[4] = {speedVal, speedVal, speedVal, speedVal};
    extern void motorApplyLayout(const int32_t speedBySlot[4]);
    motorApplyLayout(sp);
    Serial.printf("[WS-MotorTestAll] Test all motors, direction=%d\n", direction);
  } else if (strcmp(t, "motorScale") == 0) {
    // Đặt scale cho 1 bánh: payload = "slot_scale" (VD: "0_0.95")
    const char *payloadStr = doc["payload"] | "";
    int slot = -1;
    float scale = 1.0f;
    if (sscanf(payloadStr, "%d_%f", &slot, &scale) == 2) {
      if (slot >= 0 && slot < 4 && scale >= 0.5f && scale <= 1.5f) {
        extern void motorSetScale(uint8_t slot, float scale);
        motorSetScale((uint8_t)slot, scale);
        motorLayoutSaveCurrent(g_prefs);
        Serial.printf("[WS-MotorScale] Slot %d -> %.3f\n", slot, scale);
      }
    }
  } else if (strcmp(t, "motorBalance") == 0) {
    // Auto-balance: đặt tất cả bánh về cùng scale
    extern void motorAutoBalance();
    motorAutoBalance();
    motorLayoutSaveCurrent(g_prefs);
    Serial.println("[WS-MotorBalance] Auto-balanced all motors");
  } else if (strcmp(t, "motorResetScales") == 0) {
    // Reset tất cả scale về 1.0
    extern void motorResetScales();
    motorResetScales();
    motorLayoutSaveCurrent(g_prefs);
    Serial.println("[WS-MotorReset] Scales reset to 1.0");
  }
}

#endif // CTRLJSON_H
