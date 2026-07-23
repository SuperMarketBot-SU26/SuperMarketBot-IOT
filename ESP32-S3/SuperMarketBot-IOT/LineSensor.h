/* =====================================================================
 *  LineSensor.h — TCRT5000 8-channel line sensor driver (Phase 9)
 *
 *  Đọc 8 mắt TCRT5000 (analog ADC 0..4095) mỗi LINE_READ_MS (default 20ms).
 *  Tính:
 *    - activeMask: bitmask 8 sensor thấy line
 *    - offset:    -100..+100 (lệch trái/phải)
 *    - pattern:   enum LinePattern (TRACKING / NODE / JUNCTION / LOST)
 *
 *  Chia sẻ qua global g_lineState (struct LineState) để các module khác
 *  (LineDecoder, WaypointNav, RobotTelemetry) đọc atomic.
 *  Init gọi 1 lần ở setup().
 *  Update gọi ở taskControl loop (Core 1) mỗi 20ms.
 *
 *  Pattern recognition:
 *    - LOST:    0 sensor active (không thấy line)
 *    - TRACKING: 1-5 sensor active (đang bám line đơn)
 *    - JUNCTION: 3-5 sensor active ở rìa (rẽ trái hoặc phải)
 *    - NODE:    ≥6 sensor active cùng lúc (dấu +)
 *
 *  Offset tính bằng weighted average:
 *    offset = Σ (i - 3.5) * mask[i] * 100 / (active_count * 3.5)
 *  → offset = 0 khi line ở giữa
 *  → offset âm khi robot lệch trái (cần rẽ phải để về giữa)
 *  → offset dương khi robot lệch phải
 * =====================================================================*/
#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H

#include <Arduino.h>
#include <stdint.h>
#include "Config.h"

/** Pattern hiện tại của line sensor. */
enum LinePattern : uint8_t {
  LINE_PAT_UNKNOWN  = 0,
  LINE_PAT_LOST     = 1,    // 0 sensor thấy line
  LINE_PAT_TRACKING = 2,    // 1-5 sensor (bám line đơn, thẳng)
  LINE_PAT_JUNCTION = 3,    // 3-5 sensor ở rìa → rẽ nhánh T/Y
  LINE_PAT_NODE     = 4     // ≥6 sensor cùng lúc → dấu +
};

/** Snapshot trạng thái line, share toàn cục. */
struct LineState {
  uint16_t  raw[8];           // ADC raw (0..4095)
  uint8_t   activeMask;       // 8-bit, bit i = sensor i thấy line
  int16_t   offset;          // -100..+100
  LinePattern pattern;
  uint8_t   stableFrames;     // số frame liên tiếp pattern giữ nguyên (debounce)
  uint16_t  nodeCount;        // số node đã đi qua trong session
  uint32_t  lastUpdateMs;
};

extern LineState g_lineState;

/** Lưu lịch sử offset/gain cho Kalman/EKF pose fusion. */
extern float g_lineOffsetEMA;        // offset smoothed (low-pass)
extern float g_lineOffsetVariance;   // ước lượng variance cho Kalman

/** Init: cấu hình GPIO, ADC, set state mặc định. */
inline void lineSensorInit() {
#if USE_LINE_SENSOR
  // Cấu hình GPIO input + (nếu cần) pull-up
  const uint8_t pins[8] = {
    LINE_PIN_S0, LINE_PIN_S1, LINE_PIN_S2, LINE_PIN_S3,
    LINE_PIN_S4, LINE_PIN_S5, LINE_PIN_S6, LINE_PIN_S7
  };

  for (int i = 0; i < 8; i++) {
    uint8_t p = pins[i];
    if (p < 0) continue;
    // Reset GPIO pin to detach any default hardware functions (like JTAG on GPIO3)
    gpio_reset_pin((gpio_num_t)p);
    pinMode(p, INPUT);
  }


  memset(&g_lineState, 0, sizeof(g_lineState));
  g_lineState.pattern = LINE_PAT_UNKNOWN;
  g_lineOffsetEMA = 0;
  g_lineOffsetVariance = 100.0f;

  Serial.println("[LINE] TCRT5000 x8 initialized: S0..S7 = " +
    String(LINE_PIN_S0) + "," + String(LINE_PIN_S1) + "," +
    String(LINE_PIN_S2) + "," + String(LINE_PIN_S3) + "," +
    String(LINE_PIN_S4) + "," + String(LINE_PIN_S5) + "," +
    String(LINE_PIN_S6) + "," + String(LINE_PIN_S7));
#endif
}

/**
 * Đọc 8 sensor (analog), update g_lineState.
 * Tính activeMask, offset, pattern.
 * Stabilization frame: cộng dồn nếu pattern giữ nguyên, reset nếu khác.
 *
 * @return pattern hiện tại (LinePattern enum)
 */
// Ngưỡng bám vạch chuẩn 100% theo đo đạc thực tế của bạn:
//   Nền Trắng & Đặt Tay -> Raw THẤP (< 3880) -> Bit 0 (Sàn trắng)
// Ngưỡng bám vạch cài đặt theo yêu cầu của bạn: Raw > 4085 -> Bit 1 (Vạch Đen)
const uint16_t g_lineThresholds[8] = {
  4085, // S0 (Raw < 4085 -> 0 | Raw > 4085 -> 1)
  4085, // S1
  4085, // S2
  4085, // S3
  4085, // S4
  4085, // S5
  4085, // S6
  4085  // S7
};

const uint16_t g_lineHysteresis[8] = {
  10, // S0 (Hysteresis nhỏ 10 đơn vị)
  10, // S1
  10, // S2
  10, // S3
  10, // S4
  10, // S5
  10, // S6
  10  // S7
};

inline LinePattern lineSensorUpdate() {
#if !USE_LINE_SENSOR
  return LINE_PAT_UNKNOWN;
#else
  const uint8_t pins[8] = {
    LINE_PIN_S0, LINE_PIN_S1, LINE_PIN_S2, LINE_PIN_S3,
    LINE_PIN_S4, LINE_PIN_S5, LINE_PIN_S6, LINE_PIN_S7
  };

  // Đọc ADC (analog)
  uint8_t mask = 0;
  int activeCount = 0;
  int weightedSum = 0;   // dùng để phân biệt nhánh T/Y trong junction
  // Cached thresholds (sensor-by-sensor hysteresis)
  static int16_t prevThresh[8] = {0};

  for (int i = 0; i < 8; i++) {
    int raw;
    uint8_t pin = pins[i];
    if (pin < 0) {
      raw = 0; // Unused pin defaults to 0 (White floor)
    } else if (pin == 39 || pin == 48 || pin == 0 || pin > 20) {
      // Chân Digital (không thuộc ADC): đọc digitalRead (HIGH=4095, LOW=0)
      raw = digitalRead(pin) ? 4095 : 0;
    } else {
      raw = analogRead(pin);
    }
    if (raw < 0) raw = 0;
    if (raw > 4095) raw = 4095;
    g_lineState.raw[i] = (uint16_t)raw;

    // Hysteresis per sensor: NỀN TRẮNG = < 3880 (OFF) | VẠCH ĐEN = > 3880 (ON)
    uint16_t baseThresh = g_lineThresholds[i];
    uint16_t hyst = g_lineHysteresis[i];
    int16_t t = prevThresh[i];
    if (t == 0) t = baseThresh;

    if (g_lineState.activeMask & (1 << i)) {
      // Đang active (vạch đen, raw CAO) -> cần tụt xuống dưới threshold - hyst mới thành OFF (nền trắng)
      if (raw < baseThresh - hyst) {
        t = baseThresh;
      }
    } else {
      // Đang OFF (nền trắng, raw THẤP) -> cần vượt quá threshold + hyst mới thành ON (vạch đen)
      if (raw > baseThresh + hyst) {
        t = baseThresh - hyst;
      } else {
        t = baseThresh;
      }
    }
    prevThresh[i] = t;

    // Vạch Đen: Khi đè lên vạch đen raw sẽ VỌT CAO (> t)
    bool active = (raw > t);

    if (active) {
      mask |= (1 << i);
      activeCount++;
      // Cộng dồn weighted sum để phân biệt T-junction trái/phải
      weightedSum += (i - 4);  // range -4..+3
    }
  }
  g_lineState.activeMask = mask;

  // Offset tính theo trung bình trọng số tuyến tính (S0=-7 .. S7=+7)
  if (activeCount > 0) {
    float sumWeights = 0.0f;
    for (int i = 0; i < 8; i++) {
      if (mask & (1 << i)) {
        float w = (i - 3.5f) * 2.0f;  // -7, -5, -3, -1, +1, +3, +5, +7
        sumWeights += w;
      }
    }
    float avgWeight = sumWeights / (float)activeCount;  // range -7.0 .. +7.0
    float off = avgWeight * (100.0f / 7.0f);            // normalize -100 .. +100
    if (off > 100.0f) off = 100.0f;
    if (off < -100.0f) off = -100.0f;
    g_lineState.offset = (int16_t)off;

    const float alpha = 0.4f;
    g_lineOffsetEMA = alpha * off + (1.0f - alpha) * g_lineOffsetEMA;
  } else {
    g_lineState.offset = 0;
    g_lineOffsetEMA *= 0.5f;
  }

  // Pattern detection
  LinePattern newPattern;
  if (activeCount == 0) {
    newPattern = LINE_PAT_LOST;
  } else if (activeCount >= LINE_NODE_MIN_ACTIVE) {
    // ≥6 sensor → node (dấu +)
    newPattern = LINE_PAT_NODE;
  } else if (activeCount >= LINE_JUNCTION_MIN) {
    // 3-5 sensor → rẽ nhánh (T/Y junction)
    // Phân biệt trái/phải: weightedSum < 0 = line lệch trái dominant
    //                      weightedSum > 0 = line lệch phải dominant
    if (weightedSum < -2) {
      // Nhánh trái dominant → ghi nhận để rẽ trái nếu cần
      newPattern = LINE_PAT_JUNCTION;
    } else if (weightedSum > 2) {
      // Nhánh phải dominant
      newPattern = LINE_PAT_JUNCTION;
    } else {
      newPattern = LINE_PAT_JUNCTION;
    }
  } else {
    newPattern = LINE_PAT_TRACKING;
  }

  // Debounce: cần ≥LINE_NODE_DEBOUNCE_FRAMES frame giống nhau → count++
  if (newPattern == g_lineState.pattern) {
    if (g_lineState.stableFrames < 255) g_lineState.stableFrames++;
  } else {
    g_lineState.pattern = newPattern;
    g_lineState.stableFrames = 1;
  }

  // Detect node-cross event khi pattern chuyển sang NODE stable
  static LinePattern prevPattern = LINE_PAT_UNKNOWN;
  if (newPattern == LINE_PAT_NODE && g_lineState.stableFrames >= LINE_NODE_DEBOUNCE_FRAMES && prevPattern != LINE_PAT_NODE) {
    g_lineState.nodeCount++;
    Serial.printf("[LINE] Node crossed (#%u)\n", g_lineState.nodeCount);
  }
  prevPattern = newPattern;

  g_lineState.lastUpdateMs = millis();
  return g_lineState.pattern;
#endif
}

/** True khi line sensor đang thấy line (ít nhất 1 sensor active). */
inline bool lineIsOnTrack() {
  return g_lineState.pattern == LINE_PAT_TRACKING ||
         g_lineState.pattern == LINE_PAT_JUNCTION;
}

/** Snapshot sync sang g_state (volatile). Chạy định kỳ 50Hz. */
inline void lineSensorPublishState() {
#if USE_LINE_SENSOR
  g_state.lineActiveMask    = g_lineState.activeMask;
  g_state.lineOffset        = g_lineState.offset;
  g_state.linePattern       = (uint8_t)g_lineState.pattern;
  g_state.lineStableFrames  = g_lineState.stableFrames;
  g_state.lineLastUpdateMs  = g_lineState.lastUpdateMs;
  for (int i = 0; i < 8; i++) g_state.lineRaw[i] = g_lineState.raw[i];

  // In log test mỗi 1000ms để kiểm tra hoạt động của cảm biến
  static uint32_t s_lastLogMs = 0;
  uint32_t nowMs = millis();
  if (nowMs - s_lastLogMs >= 1000) {
    s_lastLogMs = nowMs;
    Serial.print(F("[LineTest] Bits: 0b"));
    for (int b = 7; b >= 0; b--) {
      Serial.print((g_lineState.activeMask & (1 << b)) ? '1' : '0');
    }
    Serial.printf(" | Offset: %d | Raw[0..7]: %u,%u,%u,%u,%u,%u,%u,%u\n",
                  g_lineState.offset,
                  g_lineState.raw[0], g_lineState.raw[1], g_lineState.raw[2], g_lineState.raw[3],
                  g_lineState.raw[4], g_lineState.raw[5], g_lineState.raw[6], g_lineState.raw[7]);
  }
#endif
}

#endif // LINE_SENSOR_H
