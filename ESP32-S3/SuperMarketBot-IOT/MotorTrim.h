/* =====================================================================
 *  MotorTrim.h — Auto-calibrate motor trim dựa trên yaw drift của IMU
 *
 *  Khi đi thẳng (cruise), IMU báo yaw drift tích lũy. Nếu robot lệch phải
 *  (yaw tăng theo convention), nghĩa là bánh Trái đang quá mạnh → giảm
 *  g_state.leftMotorScale. Ngược lại.
 *
 *  Safety:
 *   - Chỉ chạy khi IMU enabled + MODE_AUTO/MODE_WAYPOINT + cmdY ≠ 0 (đi thẳng).
 *   - Scale bị clamp [MOTOR_SCALE_MIN..MOTOR_SCALE_MAX].
 *   - Drift < AUTO_CAL_DEAD_ZONE_DEGPS → bỏ qua (nhiễu IMU).
 *   - Mỗi tick chỉ điều chỉnh AUTO_CAL_STEP (0.5%) → mượt, không giật.
 *   - Ghi NVS mỗi AUTO_CAL_SAVE_MS → tránh flash wear.
 *
 *  API:
 *    motorTrimInit(prefs)        — Load scale từ NVS, fallback = 1.0
 *    motorTrimTick(now, heading) — Gọi mỗi AUTO_CAL_INTERVAL_MS khi đi thẳng
 *    motorTrimSave(prefs)        — Force save NVS (khi tắt robot / debug)
 *    motorTrimReset(prefs)       — Reset scale = 1.0 và save
 * =====================================================================*/
#ifndef MOTOR_TRIM_H
#define MOTOR_TRIM_H

#include "Config.h"
#include <Preferences.h>

#if AUTO_CAL_ENABLE

/** Trạng thái auto-calibrate (đọc qua telemetry) */
struct MotorTrimState {
  float    lastDriftDegps   = 0.f;   // Yaw drift đo được lần cuối (deg/s)
  float    lastScaleL       = 1.f;
  float    lastScaleR       = 1.f;
  uint32_t lastTickMs       = 0;
  uint32_t lastSaveMs       = 0;
  bool     dirty            = false; // Có thay đổi scale chưa save NVS
  uint16_t adjustCount      = 0;     // Số lần đã điều chỉnh (debug)
};

extern MotorTrimState g_motorTrim;
// Header-only inline: định nghĩa ở đây để không cần file .cpp riêng.
// Tránh multi-definition bằng cách dùng 'inline' cho biến (C++17).
inline MotorTrimState& motorTrimInstance() {
  static MotorTrimState inst;
  return inst;
}

/** Helper: tính góc drift (deg) giữa 2 lần đo, có unwrap */
inline float motorTrimDeltaDeg(float hNow, float hPrev) {
  float d = hNow - hPrev;
  while (d >  180.f) d -= 360.f;
  while (d < -180.f) d += 360.f;
  return d;
}

/** Đọc scale từ NVS; nếu không có / invalid → dùng default 1.0 */
inline void motorTrimInit(Preferences &prefs) {
  // Đảm bảo range hợp lệ trước khi apply (NVS có thể bị xáo trộn khi flash wear)
  auto loadScale = [&](const char *key, float defVal) -> float {
    if (!prefs.isKey(key)) return defVal;
    float v = prefs.getFloat(key, defVal);
    if (!(v >= MOTOR_SCALE_MIN && v <= MOTOR_SCALE_MAX)) {
      Serial.printf("[Trim] NVS %s=%.3f out of range -> reset to %.2f\n",
                    key, v, defVal);
      return defVal;
    }
    return v;
  };

  g_state.leftMotorScale  = loadScale(NVS_KEY_SCALE_L, LEFT_MOTOR_SCALE_DEFAULT);
  g_state.rightMotorScale = loadScale(NVS_KEY_SCALE_R, RIGHT_MOTOR_SCALE_DEFAULT);
  motorTrimInstance().lastScaleL  = g_state.leftMotorScale;
  motorTrimInstance().lastScaleR  = g_state.rightMotorScale;

  Serial.printf("[Trim] Init L=%.3f R=%.3f (defaults L=%.2f R=%.2f)\n",
                g_state.leftMotorScale, g_state.rightMotorScale,
                LEFT_MOTOR_SCALE_DEFAULT, RIGHT_MOTOR_SCALE_DEFAULT);
}

/**
 * Gọi mỗi AUTO_CAL_INTERVAL_MS từ taskControl khi robot đang đi thẳng
 * (cmdY != 0, cmdX ≈ 0, cmdStrafe ≈ 0, MODE_AUTO hoặc MODE_WAYPOINT).
 *
 * @param now         millis() hiện tại
 * @param prevHeading heading trước đó (deg hoặc rad tùy project — Localization.h dùng rad)
 *                    NOTE: project này dùng rad → convert sang deg cho drift deg/s.
 */
inline void motorTrimTick(uint32_t now, float headingRad, float prevHeadingRad) {
  if (motorTrimInstance().lastTickMs == 0) {
    motorTrimInstance().lastTickMs = now;
    return;  // Tick đầu: chưa có delta
  }
  uint32_t dtMs = now - motorTrimInstance().lastTickMs;
  if (dtMs < AUTO_CAL_INTERVAL_MS) return;

  // Tính drift (deg/s). Lưu ý: headingRad trong project này là radian.
  float dRad = motorTrimDeltaDeg(headingRad * 57.2957795f, prevHeadingRad * 57.2957795f);
  float driftDegps = dRad * 1000.f / (float)dtMs;

  motorTrimInstance().lastDriftDegps = driftDegps;

  // Dead zone: bỏ qua drift quá nhỏ (nhiễu IMU)
  if (fabsf(driftDegps) < AUTO_CAL_DEAD_ZONE_DEGPS) {
    motorTrimInstance().lastTickMs = now;
    return;
  }

  // Drift > threshold mới kích hoạt điều chỉnh
  if (fabsf(driftDegps) < AUTO_CAL_THRESH_DEGPS) {
    motorTrimInstance().lastTickMs = now;
    return;
  }

  // Logic: drift dương = yaw tăng = robot xoay trái (= lệch phải về hướng đi).
  //       → bánh Trái quá mạnh → giảm scaleL.
  //       drift âm → robot lệch trái → tăng scaleL (hoặc giảm scaleR).
  float adjust = -driftDegps * AUTO_CAL_STEP / AUTO_CAL_THRESH_DEGPS;
  if (adjust >  AUTO_CAL_STEP) adjust =  AUTO_CAL_STEP;
  if (adjust < -AUTO_CAL_STEP) adjust = -AUTO_CAL_STEP;

  float newScaleL = g_state.leftMotorScale + adjust;
  if (newScaleL < MOTOR_SCALE_MIN) newScaleL = MOTOR_SCALE_MIN;
  if (newScaleL > MOTOR_SCALE_MAX) newScaleL = MOTOR_SCALE_MAX;

  if (newScaleL != g_state.leftMotorScale) {
    g_state.leftMotorScale = newScaleL;
    motorTrimInstance().lastScaleL = newScaleL;
    motorTrimInstance().dirty = true;
    motorTrimInstance().adjustCount++;
    Serial.printf("[Trim] drift=%.2f deg/s -> scaleL=%.3f (adj #%u)\n",
                  driftDegps, newScaleL, (unsigned)motorTrimInstance().adjustCount);
  }

  // Save NVS định kỳ (tránh flash wear)
  if (motorTrimInstance().dirty && (now - motorTrimInstance().lastSaveMs >= AUTO_CAL_SAVE_MS)) {
    Preferences p;
    if (p.begin(NVS_NAMESPACE, false)) {
      p.putFloat(NVS_KEY_SCALE_L, g_state.leftMotorScale);
      p.end();
      motorTrimInstance().lastSaveMs = now;
      motorTrimInstance().dirty = false;
      Serial.printf("[Trim] Saved L=%.3f to NVS\n", g_state.leftMotorScale);
    }
  }

  motorTrimInstance().lastTickMs = now;
}

/** Force save NVS — gọi khi tắt robot / vào MODE_MANUAL */
inline void motorTrimSave(Preferences &prefs) {
  if (!prefs.begin(NVS_NAMESPACE, false)) return;
  prefs.putFloat(NVS_KEY_SCALE_L, g_state.leftMotorScale);
  prefs.putFloat(NVS_KEY_SCALE_R, g_state.rightMotorScale);
  prefs.end();
  motorTrimInstance().lastSaveMs = (uint32_t)millis();
  motorTrimInstance().dirty = false;
  Serial.printf("[Trim] Force-saved L=%.3f R=%.3f\n",
                g_state.leftMotorScale, g_state.rightMotorScale);
}

/** Reset scale về 1.0 và save — dùng khi debug / đổi cấu hình cơ khí */
inline void motorTrimReset(Preferences &prefs) {
  g_state.leftMotorScale  = LEFT_MOTOR_SCALE_DEFAULT;
  g_state.rightMotorScale = RIGHT_MOTOR_SCALE_DEFAULT;
  motorTrimSave(prefs);
  Serial.println(F("[Trim] Reset to defaults 1.0/1.0"));
}

#else  // AUTO_CAL_ENABLE == 0

// Stubs: nếu AUTO_CAL_ENABLE=0, các API vẫn tồn tại nhưng no-op.
inline void motorTrimInit(Preferences &prefs) {
  g_state.leftMotorScale  = LEFT_MOTOR_SCALE_DEFAULT;
  g_state.rightMotorScale = RIGHT_MOTOR_SCALE_DEFAULT;
}
inline void motorTrimTick(uint32_t, float, float) {}
inline void motorTrimSave(Preferences &prefs) {
  prefs.begin(NVS_NAMESPACE, false);
  prefs.putFloat(NVS_KEY_SCALE_L, g_state.leftMotorScale);
  prefs.putFloat(NVS_KEY_SCALE_R, g_state.rightMotorScale);
  prefs.end();
}
inline void motorTrimReset(Preferences &prefs) {
  g_state.leftMotorScale  = LEFT_MOTOR_SCALE_DEFAULT;
  g_state.rightMotorScale = RIGHT_MOTOR_SCALE_DEFAULT;
  motorTrimSave(prefs);
}

#endif // AUTO_CAL_ENABLE

#endif // MOTOR_TRIM_H