/* =====================================================================
 *  LineDecoder.h — Line-following + obstacle avoidance state machine (Phase 9.1)
 *
 *  ★★★ THIẾT KẾ MỚI ★★★
 *
 *  State machine:
 *    IDLE           - chưa hoạt động (mode ≠ LINE)
 *    SEARCH_LINE    - vừa vào mode LINE → xoay tại chỗ quét vạch
 *    TRACKING       - đang bám line PID (offset → mix-steering)
 *    APPROACHING    - thấy ≥6 sensor (gần node +) → giảm tốc
 *    AT_NODE        - đến node → dừng 300ms → đếm node
 *    AVOID_OBS      - có vật cản phía trước → OA xử lý
 *    LOST_RECOVERY  - mất line trong khi đang track → xoay zig-zag tìm lại
 *    LOST_TIMEOUT   - quá 3s không tìm được → dừng + cảnh báo web
 *
 *  Steering (mix-steering):
 *    lineOffset (-100..+100) → turn (-MAX..+MAX)
 *    Không dùng full-lock (botDrive ±100) sẽ xoay tại chỗ.
 *    Mix: leftSpeed = baseSpd - turn, rightSpeed = baseSpd + turn
 *    Nếu turn = baseSpd → bên đó dừng, bên kia chạy → quay hẹp.
 *
 *  Obstacle Avoidance:
 *    Mode 1 OA: khi gặp vật phía trước (< usStopCm), chuyển sang OA.
 *    Sau khi OA xong → SEARCH_LINE để tìm lại vạch.
 *
 *  Speed slider:
 *    g_state.lineSpeedPct (0..100) — điều chỉnh qua Web UI.
 *    Map: 0% = dừng, 100% = full power. Default 60%.
 *
 *  Hysteresis:
 *    Pattern chuyển cần ≥LINE_PATTERN_DEBOUNCE_FRAMES frame ổn định.
 * =====================================================================*/
#ifndef LINE_DECODER_H
#define LINE_DECODER_H

#include "Config.h"
#include "LineSensor.h"
#include "Motors.h"
#include "Localization.h"
#include "ObstacleSensors.h"
#include "LocalObstacleAvoid.h"   // OaContext, oaTick, oaBegin, oaReset

/** Line state machine states. */
enum LineDecoderState : uint8_t {
  LD_IDLE           = 0,
  LD_SEARCH_LINE    = 1,    // xoay quét vạch lúc vừa vào mode
  LD_TRACKING       = 2,    // đang bám line
  LD_APPROACHING    = 3,    // gần node + → giảm tốc
  LD_AT_NODE        = 4,    // tại node → dừng 300ms
  LD_AVOID_OBS      = 5,    // né vật cản
  LD_LOST_RECOVERY  = 6,    // mất line giữa chừng → xoay zig-zag tìm
  LD_LOST_TIMEOUT   = 7     // quá timeout → dừng cứng, chờ user xử lý
};

/** Cấu hình PID line tracking — retune mượt mà (KP=1.15, KD=0.75) để dập tắt triệt để hiện tượng giật lắc. */
constexpr float LINE_KP = 1.15f;        // P gain — giảm nhẹ từ 1.85 về 1.15 để hết giật lắc
constexpr float LINE_KD = 0.75f;        // D gain — tăng mạnh lên 0.75 để dập tắt triệt để dao động
constexpr float LINE_MAX_TURN = 75.0f;  // turn tối đa 75% — lái cực mượt
constexpr float LINE_CRUISE_SPEED_PCT = 60.0f;   // base forward khi tracking (slider sẽ override)
constexpr float LINE_APPROACH_SPEED_PCT = 28.0f; // chậm khi gần node
constexpr float LINE_SEARCH_TURN_PCT = 45.0f;    // xoay tại chỗ khi search/recovery
constexpr float LINE_SEARCH_FWD_PCT  = 0.0f;     // BẰNG 0: Khi mất vạch tuyệt đối KHÔNG chạy thẳng tự do!

/** Timeouts (ms). */
constexpr uint32_t LD_NODE_STOP_MS         = 300u;
constexpr uint32_t LD_LOST_TIMEOUT_MS      = 3000u;
constexpr uint32_t LD_SEARCH_SPIN_MS       = 250u;   // 1 nửa zig-zag = 250ms
constexpr uint32_t LD_RECOVERY_ZIGZAG_MS   = 400u;   // zig zag step

extern LineDecoderState g_ldState;
extern uint32_t g_ldStateEnterMs;
extern uint32_t g_lostSinceMs;
extern uint8_t  g_lineSpeedPct;   // 0..100 — slider từ web

static inline const char* ldStateName(LineDecoderState s) {
  switch (s) {
    case LD_IDLE:           return "IDLE";
    case LD_SEARCH_LINE:    return "SEARCH";
    case LD_TRACKING:       return "TRACK";
    case LD_APPROACHING:    return "APPROACH";
    case LD_AT_NODE:        return "AT_NODE";
    case LD_AVOID_OBS:      return "AVOID_OBS";
    case LD_LOST_RECOVERY:  return "LOST_REC";
    case LD_LOST_TIMEOUT:   return "LOST_TMO";
    default:                return "?";
  }
}

inline void lineDecoderInit() {
  g_ldState = LD_IDLE;
  g_ldStateEnterMs = millis();
  g_lostSinceMs = 0;
}

inline void lineDecoderSetState(LineDecoderState s) {
  if (s != g_ldState) {
    Serial.printf("[LINE] State %s → %s\n", ldStateName(g_ldState), ldStateName(s));
    g_ldState = s;
    g_ldStateEnterMs = millis();
    if (s == LD_LOST_RECOVERY || s == LD_LOST_TIMEOUT) {
      g_lostSinceMs = millis();
    }
  }
}

/**
 * Tính turn cmd từ offset hiện tại (PD).
 * Input:  offset (-100..+100)
 * Output: turn (-LINE_MAX_TURN..+LINE_MAX_TURN)
 *         - turn dương = line lệch phải → robot cần rẽ phải → right chậm, left nhanh
 *         - (theo convention: offset > 0 = line ở bên phải robot → cần quay phải)
 */
inline int16_t lineComputeTurn(int16_t offset, float dtS) {
  static int16_t prevOffset = 0;
  static bool   first = true;
  int16_t d;
  if (first) {
    d = 0;
    first = false;
  } else {
    d = (int16_t)(offset - prevOffset);
  }
  prevOffset = offset;

  float turn = LINE_KP * (float)offset + LINE_KD * (d / (dtS > 0.001f ? dtS : 0.02f));
  if (turn >  LINE_MAX_TURN) turn =  LINE_MAX_TURN;
  if (turn < -LINE_MAX_TURN) turn = -LINE_MAX_TURN;
  return (int16_t)turn;
}

/** Lấy tốc độ forward (0..100%) từ slider web, kẹp biên an toàn. */
inline int16_t lineGetForwardPct() {
  int16_t v = (int16_t)g_lineSpeedPct;
  if (v < 15) v = 15;   // tối thiểu 15% để motor có đủ torque
  if (v > 100) v = 100;
  return v;
}

/** Điều khiển xe theo turn + forward (tăng đà đủ torque để chạy trên sàn nhà có ma sát). */
inline void lineBotDrive(int16_t turn, int16_t fwdPct) {
  uint16_t baseSpd = g_state.baseSpeed;
  if (baseSpd == 0) baseSpd = PWM_MAX; // Dùng 100% PWM_MAX làm base để không bị thóp công suất
  
  // Tăng fwdPct tuyến tính (tối thiểu 75%) để vượt qua lực ma sát ma sát bánh xe trên sàn
  int16_t boostedFwd = fwdPct;
  if (boostedFwd > 0 && boostedFwd < 75) boostedFwd = 75; // Đảm bảo lực kéo tối thiểu trên sàn
  
  botDrive(turn, boostedFwd, baseSpd);
}

/** Reset OA context khi vào/chuyển chế độ. */
inline void lineResetOA() {
#if USE_HC_SR04_HARDWARE || !USE_LIDAR_HARDWARE
  oaReset(g_oaCtx);
#endif
}

/** Cập nhật line steering — gọi mỗi SAFE_LOOP_MS (~50ms). */
inline void lineDecoderUpdate(float dtS) {
#if !USE_LINE_SENSOR
  return;
#else
  if (g_state.mode != MODE_LINE) {
    if (g_ldState != LD_IDLE) {
      botStop();
      lineDecoderSetState(LD_IDLE);
    }
    return;
  }

  LinePattern p = g_lineState.pattern;
  int16_t offset = g_lineState.offset;
  int16_t fwd = lineGetForwardPct();

  // Khi mới vào mode LINE từ mode khác → SEARCH
  if (g_ldState == LD_IDLE) {
    lineDecoderSetState(LD_SEARCH_LINE);
    return;
  }

  switch (g_ldState) {

    /* ── SEARCH_LINE: vừa vào mode → xoay quét vạch ─────────────── */
    case LD_SEARCH_LINE: {
      // Có vật cản sát phía trước (<= 10cm) → né trước
      int16_t frontCm = obsFrontCm();
      if (obsCmValid(frontCm) && frontCm <= 10) {
        lineDecoderSetState(LD_AVOID_OBS);
        lineResetOA();
        break;
      }
      // Thấy vạch → track ngay
      if (p != LINE_PAT_LOST) {
        lineDecoderSetState(LD_TRACKING);
        Serial.printf("[LINE] Found line! pattern=%d mask=0x%02X\n",
                      (int)p, g_lineState.activeMask);
        break;
      }
      // Xoay zig-zag tại chỗ (đổi chiều mỗi LD_SEARCH_SPIN_MS)
      uint32_t elapsed = millis() - g_ldStateEnterMs;
      bool turnRight = ((elapsed / LD_SEARCH_SPIN_MS) & 1u) == 0u;
      // Đi tới nhẹ để robot tự rời vị trí
      int16_t turnCmd = turnRight ? (int16_t)LINE_SEARCH_TURN_PCT
                                  : -(int16_t)LINE_SEARCH_TURN_PCT;
      lineBotDrive(turnCmd, (int16_t)LINE_SEARCH_FWD_PCT);

      // Timeout → LOST_TIMEOUT
      if (millis() - g_ldStateEnterMs > 5000u) {
        lineDecoderSetState(LD_LOST_TIMEOUT);
      }
      break;
    }

    /* ── TRACKING: bám line bình thường ───────────────────────────── */
    case LD_TRACKING: {
      // OA: vật cản sát phía trước (chỉ trigger khi vật cản <= 10cm)
      int16_t frontCm = obsFrontCm();
      if (obsCmValid(frontCm) && frontCm <= 10) {
        lineDecoderSetState(LD_AVOID_OBS);
        lineResetOA();
        break;
      }
      // Mất vạch → recovery
      if (p == LINE_PAT_LOST) {
        lineDecoderSetState(LD_LOST_RECOVERY);
        break;
      }
      // Gần node + → APPROACHING (cần ổn định ≥4 frame để tránh dính node giả)
      if (p == LINE_PAT_NODE && g_lineState.stableFrames >= 4) {
        lineDecoderSetState(LD_APPROACHING);
        break;
      }
      // Steering PID bình thường
      int16_t turn = lineComputeTurn(offset, dtS);
      lineBotDrive(turn, fwd);
      break;
    }

    /* ── APPROACHING: gần node, giảm tốc ─────────────────────────── */
    case LD_APPROACHING: {
      if (obsFrontBlocked()) {
        lineDecoderSetState(LD_AVOID_OBS);
        lineResetOA();
        break;
      }
      if (p == LINE_PAT_LOST) {
        lineDecoderSetState(LD_LOST_RECOVERY);
        break;
      }
      int16_t turn = lineComputeTurn(offset, dtS);
      int16_t slow = (int16_t)LINE_APPROACH_SPEED_PCT;
      if (slow > fwd) slow = fwd;   // không bao giờ vượt slider
      lineBotDrive(turn, slow);

      // Pattern stable NODE → AT_NODE
      if (p == LINE_PAT_NODE && g_lineState.stableFrames >= LINE_NODE_DEBOUNCE_FRAMES) {
        g_state.lastNodeId++;
        g_lineState.nodeCount++;
        lineDecoderSetState(LD_AT_NODE);
        Serial.printf("[LINE] Node #%u crossed\n", g_lineState.nodeCount);
      }
      break;
    }

    /* ── AT_NODE: dừng tại node 300ms ────────────────────────────── */
    case LD_AT_NODE: {
      botDrive(0, 0, g_state.baseSpeed);
      if (millis() - g_ldStateEnterMs > LD_NODE_STOP_MS) {
        // Sau node: tiếp tục tracking
        lineDecoderSetState(LD_TRACKING);
      }
      break;
    }

    /* ── AVOID_OBS: né vật cản (OA Module 1) ─────────────────────── */
    case LD_AVOID_OBS: {
#if USE_HC_SR04_HARDWARE || !USE_LIDAR_HARDWARE
      int16_t frontCm = obsFrontCm();
      // Trigger OA: nếu chưa bắt đầu → oaBegin()
      if (g_oaCtx.state == OA_IDLE) {
        if (oaBegin(g_oaCtx, frontCm, millis())) {
          Serial.println(F("[LINE-OA] Bắt đầu né vật cản"));
        } else {
          // Không trigger được (xa quá) → về SEARCH_LINE
          lineDecoderSetState(LD_SEARCH_LINE);
          break;
        }
      }
      OaTickResult res = oaTick(g_oaCtx, frontCm, millis());
      if (res == OA_RES_DONE) {
        Serial.println(F("[LINE-OA] Né xong, quay lại tìm line"));
        // Reset OA cho lần sau
        oaReset(g_oaCtx);
        lineDecoderSetState(LD_SEARCH_LINE);
      } else if (res == OA_RES_BLOCKED) {
        Serial.println(F("[LINE-OA] Kẹt cứng → dừng"));
        oaReset(g_oaCtx);
        lineDecoderSetState(LD_LOST_TIMEOUT);
      }
#else
      // Không có OA lib → quay đầu tại chỗ 600ms (chọn chiều dựa trên offset)
      int16_t turnCmd = (offset >= 0) ? 50 : -50;
      lineBotDrive(turnCmd, 0);
      if (millis() - g_ldStateEnterMs > 600u) {
        lineDecoderSetState(LD_SEARCH_LINE);
      }
#endif
      break;
    }

    /* ── LOST_RECOVERY: mất line → xoay zig-zag tìm lại ──────────── */
    case LD_LOST_RECOVERY: {
      // OA: nếu gặp vật khi đang tìm line
      if (obsFrontBlocked()) {
        lineDecoderSetState(LD_AVOID_OBS);
        lineResetOA();
        break;
      }
      // Tìm thấy → TRACKING
      if (p != LINE_PAT_LOST) {
        Serial.println(F("[LINE] Recovery: found line"));
        lineDecoderSetState(LD_TRACKING);
        break;
      }
      // Zig-zag tại chỗ (đổi chiều mỗi LD_RECOVERY_ZIGZAG_MS)
      uint32_t elapsed = millis() - g_ldStateEnterMs;
      bool turnRight = ((elapsed / LD_RECOVERY_ZIGZAG_MS) & 1u) == 0u;
      int16_t turnCmd = turnRight ? (int16_t)LINE_SEARCH_TURN_PCT
                                  : -(int16_t)LINE_SEARCH_TURN_PCT;
      lineBotDrive(turnCmd, 0);  // xoay tại chỗ, không tới

      // Timeout → LOST_TIMEOUT
      if (millis() - g_lostSinceMs > LD_LOST_TIMEOUT_MS) {
        Serial.println(F("[LINE] Lost timeout — stopping"));
        lineDecoderSetState(LD_LOST_TIMEOUT);
      }
      break;
    }

    /* ── LOST_TIMEOUT: dừng cứng + cảnh báo ──────────────────────── */
    case LD_LOST_TIMEOUT: {
      botStop();
      // Không tự đi tiếp — chờ user can thiệp (chuyển mode khác hoặc đặt lại vị trí)
      break;
    }
  }

  /* ── Telemetry debug mỗi 300ms (đã tắt theo yêu cầu) ──────────────
  static uint32_t s_logMs = 0;
  uint32_t nowMs = millis();
  if (nowMs - s_logMs >= 300u) {
    s_logMs = nowMs;
    const char* patName = (p == LINE_PAT_LOST)     ? "LOST"
                        : (p == LINE_PAT_TRACKING)  ? "LINE"
                        : (p == LINE_PAT_JUNCTION)  ? "JUNC"
                        : (p == LINE_PAT_NODE)      ? "NODE"
                        : "?";
    Serial.printf("[LINE] %-9s pat=%-4s off=%+4d bits=0b",
                  ldStateName(g_ldState), patName, offset);
    for (int b = 7; b >= 0; b--) {
      Serial.print((g_lineState.activeMask & (1 << b)) ? '1' : '0');
    }
    Serial.printf(" fwd=%d%% front=%dcm\n", (int)fwd, (int)obsFrontCm());
  }
  */
#endif
}

#endif // LINE_DECODER_H