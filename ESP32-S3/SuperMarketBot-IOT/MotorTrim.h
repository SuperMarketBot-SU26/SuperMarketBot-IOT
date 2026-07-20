/* =====================================================================
 *  MotorTrim.h — Cân bằng 4 động cơ + Auto-trim + Slip detection
 *
 *  Cải tiến từ bản cũ:
 *    1. Scale riêng cho từng bánh (FL, RL, FR, RR)
 *    2. Auto-trim dựa trên IMU yaw drift
 *    3. Slip detection - phát hiện bánh trượt
 *    4. Cross-motor balancing - so sánh tốc độ bánh cùng bên
 *
 *  Safety:
 *   - Chỉ chạy khi IMU enabled + đi thẳng
 *   - Scale bị clamp [MOTOR_SCALE_MIN..MOTOR_SCALE_MAX]
 *   - Drift < dead zone → bỏ qua (nhiễu IMU)
 *   - Mỗi tick chỉ điều chỉnh STEP (0.5%) → mượt
 *
 * =====================================================================*/
#ifndef MOTOR_TRIM_H
#define MOTOR_TRIM_H

#include "Config.h"
#include <Preferences.h>

/** Trạng thái cân bằng */
struct MotorTrimState {
  // Scale cho 4 bánh riêng biệt
  float scaleFL = 1.0f;   // Trái trước
  float scaleRL = 1.0f;   // Trái sau
  float scaleFR = 1.0f;   // Phải trước
  float scaleRR = 1.0f;   // Phải sau

  // Scale gốc (Trái/Phải - backward compat)
  float scaleL = 1.0f;
  float scaleR = 1.0f;

  // Yaw drift tracking
  float lastDriftDegps = 0.f;
  uint32_t lastTickMs = 0;
  uint32_t lastSaveMs = 0;
  bool dirty = false;
  uint16_t adjustCount = 0;

  // Slip detection
  uint8_t slipCountFL = 0, slipCountRL = 0;
  uint8_t slipCountFR = 0, slipCountRR = 0;
  bool slipDetected = false;

  // Motor speed tracking (RPM từ encoder hoặc ước lượng)
  float lastRpmFL = 0, lastRpmRL = 0;
  float lastRpmFR = 0, lastRpmRR = 0;
};

extern MotorTrimState g_motorTrim;
MotorTrimState g_motorTrim = {};

inline MotorTrimState& motorTrimInstance() {
  static MotorTrimState inst;
  return inst;
}

/** Helper: tính góc drift (deg) giữa 2 lần đo */
inline float motorTrimDeltaDeg(float hNow, float hPrev) {
  float d = hNow - hPrev;
  while (d >  180.f) d -= 360.f;
  while (d < -180.f) d += 360.f;
  return d;
}

/** Đọc scale từ NVS */
inline void motorTrimInit(Preferences &prefs) {
  auto loadScale = [&](const char *key, float defVal) -> float {
    if (!prefs.isKey(key)) return defVal;
    float v = prefs.getFloat(key, defVal);
    if (!(v >= MOTOR_SCALE_MIN && v <= MOTOR_SCALE_MAX)) {
      Serial.printf("[Trim] NVS %s=%.3f out of range -> reset\n", key, v);
      return defVal;
    }
    return v;
  };

  // Load 4 bánh riêng
  g_motorTrim.scaleFL = loadScale("motScFL", 1.0f);
  g_motorTrim.scaleRL = loadScale("motScRL", 1.0f);
  g_motorTrim.scaleFR = loadScale("motScFR", 1.0f);
  g_motorTrim.scaleRR = loadScale("motScRR", 1.0f);

  // Load 2 bên (backward compat)
  g_motorTrim.scaleL = loadScale(NVS_KEY_SCALE_L, 1.0f);
  g_motorTrim.scaleR = loadScale(NVS_KEY_SCALE_R, 1.0f);

  Serial.printf("[Trim] Init 4-banh: FL=%.3f RL=%.3f FR=%.3f RR=%.3f\n",
                g_motorTrim.scaleFL, g_motorTrim.scaleRL,
                g_motorTrim.scaleFR, g_motorTrim.scaleRR);
  Serial.printf("[Trim] Init 2-ben: L=%.3f R=%.3f\n",
                g_motorTrim.scaleL, g_motorTrim.scaleR);

  // Ghi vào g_state để Motors.h dùng
  g_state.leftMotorScale  = g_motorTrim.scaleL;
  g_state.rightMotorScale = g_motorTrim.scaleR;
}

/**
 * Gọi mỗi AUTO_CAL_INTERVAL_MS khi robot đang đi thẳng (5 tham số - đầy đủ)
 */
inline void motorTrimTick(uint32_t now, float headingRad, float prevHeadingRad,
                           int16_t cmdY, int16_t cmdX) {
  // Chỉ chạy khi đi thẳng (cmdY != 0, cmdX ≈ 0)
  if (abs(cmdY) < 30 || abs(cmdX) > 20) {
    motorTrimInstance().lastTickMs = now;
    return;
  }

  if (motorTrimInstance().lastTickMs == 0) {
    motorTrimInstance().lastTickMs = now;
    return;
  }

  uint32_t dtMs = now - motorTrimInstance().lastTickMs;
  if (dtMs < AUTO_CAL_INTERVAL_MS) return;

  // Tính drift (deg/s)
  float dDeg = motorTrimDeltaDeg(headingRad * 57.2957795f, prevHeadingRad * 57.2957795f);
  float driftDegps = dDeg * 1000.f / (float)dtMs;
  motorTrimInstance().lastDriftDegps = driftDegps;

  // Dead zone: bỏ qua drift quá nhỏ
  if (fabsf(driftDegps) < AUTO_CAL_DEAD_ZONE_DEGPS) {
    motorTrimInstance().lastTickMs = now;
    return;
  }

  // Threshold check
  if (fabsf(driftDegps) < AUTO_CAL_THRESH_DEGPS) {
    motorTrimInstance().lastTickMs = now;
    return;
  }

#if AUTO_CAL_ENABLE
  // Logic: drift dương = yaw tăng = robot xoay trái = bánh Trái mạnh → giảm scaleL
  float adjust = -driftDegps * AUTO_CAL_STEP / AUTO_CAL_THRESH_DEGPS;
  if (adjust >  AUTO_CAL_STEP) adjust =  AUTO_CAL_STEP;
  if (adjust < -AUTO_CAL_STEP) adjust = -AUTO_CAL_STEP;

  float newScaleL = g_state.leftMotorScale + adjust;
  if (newScaleL < MOTOR_SCALE_MIN) newScaleL = MOTOR_SCALE_MIN;
  if (newScaleL > MOTOR_SCALE_MAX) newScaleL = MOTOR_SCALE_MAX;

  if (fabsf(newScaleL - g_state.leftMotorScale) > 0.0001f) {
    g_state.leftMotorScale = newScaleL;
    g_motorTrim.scaleL = newScaleL;
    motorTrimInstance().dirty = true;
    motorTrimInstance().adjustCount++;
    Serial.printf("[Trim] drift=%.2f deg/s -> scaleL=%.3f (#%u)\n",
                  driftDegps, newScaleL, (unsigned)motorTrimInstance().adjustCount);
  }

  // Save NVS định kỳ
  if (motorTrimInstance().dirty && (now - motorTrimInstance().lastSaveMs >= AUTO_CAL_SAVE_MS)) {
    Preferences p;
    if (p.begin(NVS_NAMESPACE, false)) {
      p.putFloat(NVS_KEY_SCALE_L, g_state.leftMotorScale);
      p.end();
      motorTrimInstance().lastSaveMs = now;
      motorTrimInstance().dirty = false;
    }
  }
#endif

  motorTrimInstance().lastTickMs = now;
}

/**
 * Wrapper 3 tham số cho backward compatibility
 */
inline void motorTrimTickSimple(uint32_t now, float headingRad, float prevHeadingRad) {
  motorTrimTick(now, headingRad, prevHeadingRad, 50, 0);
}

/**
 * Cross-motor balancing - cân bằng FL vs RL và FR vs RR
 * Gọi mỗi tick khi robot đang chạy
 * @param actualRpmFL, actualRpmRL, actualRpmFR, actualRpmRR - RPM thực tế
 * @param targetPwm - PWM mục tiêu (để so sánh)
 */
inline void motorTrimCrossBalance(float actualRpmFL, float actualRpmRL,
                                   float actualRpmFR, float actualRpmRR,
                                   int32_t targetPwm) {
  if (targetPwm < 100) return;  // Chỉ balance khi đủ tải

  const float TOLERANCE = 0.15f;  // 15% tolerance

  // So sánh Trái trước vs Trái sau
  if (actualRpmFL > 10 && actualRpmRL > 10) {
    float ratio = actualRpmFL / actualRpmRL;
    if (ratio > 1.0f + TOLERANCE) {
      // FL nhanh hơn → giảm FL hoặc tăng RL
      float adjust = (ratio - 1.0f) * 0.01f;  // 1% adjustment max
      g_motorTrim.scaleFL = max(MOTOR_SCALE_MIN, g_motorTrim.scaleFL - adjust);
      Serial.printf("[Trim] Cross: FL>RL ratio=%.2f -> FL=%.3f\n", ratio, g_motorTrim.scaleFL);
    } else if (ratio < 1.0f - TOLERANCE) {
      // RL nhanh hơn → giảm RL hoặc tăng FL
      float adjust = ((1.0f - ratio)) * 0.01f;
      g_motorTrim.scaleFL = min(MOTOR_SCALE_MAX, g_motorTrim.scaleFL + adjust);
      Serial.printf("[Trim] Cross: RL>FL ratio=%.2f -> FL=%.3f\n", ratio, g_motorTrim.scaleFL);
    }
  }

  // So sánh Phải trước vs Phải sau
  if (actualRpmFR > 10 && actualRpmRR > 10) {
    float ratio = actualRpmFR / actualRpmRR;
    if (ratio > 1.0f + TOLERANCE) {
      float adjust = (ratio - 1.0f) * 0.01f;
      g_motorTrim.scaleFR = max(MOTOR_SCALE_MIN, g_motorTrim.scaleFR - adjust);
      Serial.printf("[Trim] Cross: FR>RR ratio=%.2f -> FR=%.3f\n", ratio, g_motorTrim.scaleFR);
    } else if (ratio < 1.0f - TOLERANCE) {
      float adjust = ((1.0f - ratio)) * 0.01f;
      g_motorTrim.scaleFR = min(MOTOR_SCALE_MAX, g_motorTrim.scaleFR + adjust);
      Serial.printf("[Trim] Cross: RR>FR ratio=%.2f -> FR=%.3f\n", ratio, g_motorTrim.scaleFR);
    }
  }
}

/**
 * Slip detection - phát hiện bánh trượt
 * So sánh RPM kỳ vọng vs RPM thực tế
 * @return true nếu phát hiện trượt
 */
inline bool motorTrimCheckSlip(float actualRpm, float expectedRpm, uint8_t &slipCount) {
  const float SLIP_THRESHOLD = 0.4f;  // 40% chênh lệch = trượt
  const uint8_t SLIP_CONFIRM = 3;      // 3 lần liên tiếp = trượt thật

  if (expectedRpm < 20) {
    slipCount = 0;
    return false;
  }

  float diff = fabsf(actualRpm - expectedRpm) / expectedRpm;

  if (diff > SLIP_THRESHOLD) {
    slipCount++;
    if (slipCount >= SLIP_CONFIRM) {
      motorTrimInstance().slipDetected = true;
      return true;
    }
  } else {
    if (slipCount > 0) slipCount--;
  }

  return false;
}

/**
 * Reset slip detection flag
 */
inline void motorTrimResetSlip() {
  motorTrimInstance().slipDetected = false;
}

/** Force save NVS */
inline void motorTrimSave(Preferences &prefs) {
  if (!prefs.begin(NVS_NAMESPACE, false)) return;
  
  // Save 4 bánh
  prefs.putFloat("motScFL", g_motorTrim.scaleFL);
  prefs.putFloat("motScRL", g_motorTrim.scaleRL);
  prefs.putFloat("motScFR", g_motorTrim.scaleFR);
  prefs.putFloat("motScRR", g_motorTrim.scaleRR);
  
  // Save 2 bên (backward compat)
  prefs.putFloat(NVS_KEY_SCALE_L, g_state.leftMotorScale);
  prefs.putFloat(NVS_KEY_SCALE_R, g_state.rightMotorScale);
  
  prefs.end();
  motorTrimInstance().lastSaveMs = (uint32_t)millis();
  motorTrimInstance().dirty = false;
  Serial.printf("[Trim] Saved: FL=%.3f RL=%.3f FR=%.3f RR=%.3f\n",
                g_motorTrim.scaleFL, g_motorTrim.scaleRL,
                g_motorTrim.scaleFR, g_motorTrim.scaleRR);
}

/** Reset scale về 1.0 */
inline void motorTrimReset(Preferences &prefs) {
  g_motorTrim.scaleFL = g_motorTrim.scaleRL = 1.0f;
  g_motorTrim.scaleFR = g_motorTrim.scaleRR = 1.0f;
  g_motorTrim.scaleL = g_motorTrim.scaleR = 1.0f;
  g_state.leftMotorScale = g_state.rightMotorScale = 1.0f;
  motorTrimSave(prefs);
  Serial.println(F("[Trim] Reset all scales to 1.0"));
}

/** Cập nhật scale từ web UI */
inline void motorTrimSetWheelScale(uint8_t slot, float scale) {
  scale = constrain(scale, MOTOR_SCALE_MIN, MOTOR_SCALE_MAX);
  
  switch(slot) {
    case 0: g_motorTrim.scaleFL = scale; break;
    case 1: g_motorTrim.scaleRL = scale; break;
    case 2: g_motorTrim.scaleFR = scale; break;
    case 3: g_motorTrim.scaleRR = scale; break;
    default: return;
  }
  
  // Cập nhật 2 bên từ 4 bánh
  g_motorTrim.scaleL = (g_motorTrim.scaleFL + g_motorTrim.scaleRL) / 2.0f;
  g_motorTrim.scaleR = (g_motorTrim.scaleFR + g_motorTrim.scaleRR) / 2.0f;
  g_state.leftMotorScale = g_motorTrim.scaleL;
  g_state.rightMotorScale = g_motorTrim.scaleR;
  
  motorTrimInstance().dirty = true;
}

/** Lấy scale cho slot cụ thể */
inline float motorTrimGetScale(uint8_t slot) {
  switch(slot) {
    case 0: return g_motorTrim.scaleFL;
    case 1: return g_motorTrim.scaleRL;
    case 2: return g_motorTrim.scaleFR;
    case 3: return g_motorTrim.scaleRR;
    default: return 1.0f;
  }
}

/** Auto-balance all motors to equal speed */
inline void motorTrimAutoBalance() {
  // Tính trung bình
  float avgFL_RL = (g_motorTrim.scaleFL + g_motorTrim.scaleRL) / 2.0f;
  float avgFR_RR = (g_motorTrim.scaleFR + g_motorTrim.scaleRR) / 2.0f;
  
  // Cân bằng 2 bên
  float avgAll = (g_motorTrim.scaleFL + g_motorTrim.scaleRL + 
                  g_motorTrim.scaleFR + g_motorTrim.scaleRR) / 4.0f;
  
  // Điều chỉnh để tất cả = trung bình
  g_motorTrim.scaleFL = g_motorTrim.scaleRL = avgAll;
  g_motorTrim.scaleFR = g_motorTrim.scaleRR = avgAll;
  
  // Cập nhật g_state
  g_state.leftMotorScale = avgAll;
  g_state.rightMotorScale = avgAll;
  g_motorTrim.scaleL = g_motorTrim.scaleR = avgAll;
  
  Serial.printf("[Trim] Auto-balanced all to %.3f\n", avgAll);
}

#endif // MOTOR_TRIM_H
