/* =====================================================================
 *  CtrlJson.h — Lệnh điều khiển JSON (WebSocket dashboard)
 * =====================================================================*/
#ifndef CTRLJSON_H
#define CTRLJSON_H

#include "Config.h"
// Motors.h bị loại khỏi đây vì kéo theo MotorLayout.h → WebSocketsServer → WiFi
// botStop() được extern định nghĩa trong Motors.h (đã include ở .ino)
extern void botStop();
#include "Localization.h"     // [Bước 6 - 2026-07-27] Bắt buộc include TRỰC TIẾP trong CtrlJson.h (không chỉ qua WaypointNav.h) để inline functions (locSetSlamPose, locResetPose) visible.
#include "WaypointNav.h"

#include <ArduinoJson.h>
#include <Preferences.h>
#include <freertos/FreeRTOS.h>


#include <freertos/semphr.h>
#include <cstring>

extern RobotState g_state;
extern SemaphoreHandle_t g_stateMutex;
extern Preferences g_prefs;

// Forward declarations cho AutoExplore (định nghĩa trong AutoExplore.h)
namespace autoExplore {
  inline void start();
  inline void stop();
  inline bool isActive();
}

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
    // Đánh dấu "joystick còn tươi VÀ đang nhấn" — cmd_vel_callback
    // (MicroRos.h) dùng giá trị này để gate /cmd_vel từ ROS2. Cập nhật
    // CÙNG critical section với cmdX/Y/Strafe để Core 1 đọc timestamp
    // nhất quán với giá trị cmdX/Y/Strafe.
    // Chỉ đánh dấu "đang nhấn" khi ít nhất một trục khác 0 — WebUI
    // publish joy ở ~10Hz kể cả khi không nhấn; nếu cứ refresh thì gate
    // ROS2 Nav2 vĩnh viễn đóng.
    if (g_state.cmdX != 0 || g_state.cmdY != 0 || g_state.cmdStrafe != 0) {
      g_state.joyLastMs = millis();
    }
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
  } else if (strcmp(t, "spdWaypoint") == 0) {
    uint16_t pct = doc["v"].as<uint16_t>();
    if (pct > 100) pct = 100;
    if (pct < 15) pct = 15;  // tối thiểu 15% cho motor torque
    g_state.waypointBaseSpeed = (uint16_t)((uint32_t)pct * PWM_MAX / 100);
    g_prefs.begin(NVS_NAMESPACE, false);
    g_prefs.putUInt("waypointSpeed", g_state.waypointBaseSpeed);
    g_prefs.end();
    Serial.printf("[WP-SPD] Slider -> %u%%\n", (unsigned)pct);
  } else if (strcmp(t, "spdSwerve") == 0) {
    uint16_t pct = doc["v"].as<uint16_t>();
    if (pct > 100) pct = 100;
    g_state.swerveBaseSpeed = (uint16_t)((uint32_t)pct * PWM_MAX / 100);
    g_prefs.begin(NVS_NAMESPACE, false);
    g_prefs.putUInt("swerveSpeed", g_state.swerveBaseSpeed);
    g_prefs.end();
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
    if (m > MODE_WAYPOINT) m = MODE_MANUAL;

    // [Bước 4 - 2026-07-27] EStop guard: không cho vào Auto/Waypoint khi EStop active.
    if (g_state.estop && m != MODE_MANUAL) {
      Serial.println(F("[WS-Mode] EStop ACTIVE — bỏ qua lệnh, yêu cầu release EStop trước."));
      return;
    }

    if (m == MODE_MANUAL) {
      robotForceManualStop();
      autoExplore::stop();   // dừng AutoExplore nếu đang chạy
    } else {
      // Đảm bảo stop AutoExplore nếu đang chạy trước khi đổi mode khác
      if (m == MODE_AUTO_EXPLORE) {
        autoExplore::start();   // bắt đầu session mới
      } else {
        autoExplore::stop();    // dừng nếu đang chạy
      }
      g_state.mode = (RobotMode)m;
      g_state.cmdX = 0;
      g_state.cmdY = 0;
      g_state.cmdStrafe = 0;
      botStop();
      if (m == MODE_WAYPOINT) {
        // [Bước 4 - 2026-07-27] Bỏ fake waypoint (1,0):
        // Không tự ý chạy khi route rỗng — yêu cầu load route từ BE / WebManager.
        if (s_wpCount == 0) {
          Serial.println(F("[WS-Mode] MODE_WAYPOINT yêu cầu nhưng route rỗng — đợi lệnh 'navigate' từ BE."));
          g_state.mode = MODE_MANUAL;  // fallback manual, không tự chạy
          // TODO: publish MQTT request_route → BE trả route
        } else {
          wpNavStart();
        }
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
  } else if (strcmp(t, "mode_auto_explore") == 0) {
    Serial.println(F("[WS-Mode] Bắt đầu MODE_AUTO_EXPLORE (Tự đi quét Map)"));
    robotForceManualStop();
    g_state.mode = MODE_AUTO_EXPLORE;
    autoExplore::start();
  } else if (strcmp(t, "scan_stop") == 0) {
    Serial.println(F("[WS-Mode] Dừng MODE_AUTO_EXPLORE"));
    autoExplore::stop();
    robotForceManualStop();
    g_state.mode = MODE_MANUAL;
  } else if (strcmp(t, "snap90") == 0) {
    int deg = doc["deg"] | 15;
    if (deg < 0) deg = 0;
    if (deg > 45) deg = 45;
    // Chuyển deg → rad rồi override define runtime (không save NVS - chỉ runtime)
    extern float g_wpSnap90TolRad;   // defined in WaypointNav.h
    g_wpSnap90TolRad = (float)deg * (float)M_PI / 180.0f;
    Serial.printf("[WS-Snap90] Đã set tolerance = %d° (%.3f rad)\n", deg, g_wpSnap90TolRad);
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
