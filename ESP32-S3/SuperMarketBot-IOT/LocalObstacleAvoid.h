/* =====================================================================
 *  LocalObstacleAvoid.h — Né vật cản THÔNG MINH
 *  
 *  Cải tiến:
 *    1. Đánh giá 4 hướng: trước, sau, trái, phải
 *    2. Ưu tiên thoát về phía TRƯỚC (waypoint)
 *    3. Quyết định: lùi, quẹo, xoay tại chỗ
 *    4. Quay lại đi thẳng sau khi thoát
 *    5. Tự động quay lại heading ban đầu
 * =====================================================================*/
#ifndef LOCAL_OBSTACLE_AVOID_H
#define LOCAL_OBSTACLE_AVOID_H

#include "Config.h"
#include "ObstacleSensors.h"
#include "Localization.h"
#include "PidController.h"
#include "Motors.h"
#include <math.h>

extern uint16_t autoSpeedPwm();

/* ==================== FSM STATES ==================== */
enum OaFsmState : uint8_t {
  OA_IDLE = 0,           // Bình thường, đi thẳng
  OA_EVALUATE,           // Đánh giá 4 hướng
  OA_WAIT_DYNAMIC,       // Chờ vật cản động (người) di chuyển
  OA_STRAIGHTEN,         // Quay về heading ban đầu
  OA_BACKUP,             // Lùi lại
  OA_TURN_TO_CLEAR,      // Xoay đến hướng trống
  OA_SWERVE_PASS,        // Trượt ngang + tiến để thoát
  OA_RESUME_CRUISE,      // Quay lại đi thẳng
  OA_BLOCKED             // Kẹt cứng - cần can thiệp
};

enum OaTickResult : uint8_t {
  OA_RES_RUNNING = 0,
  OA_RES_DONE,           // Đã thoát, tiếp tục cruise
  OA_RES_BLOCKED,        // Kẹt cứng
  OA_RES_IDLE            // Không có vật cản
};

enum EscapePriority : uint8_t {
  ESCAPE_STRAIGHT = 0,   // Ưu tiên đi thẳng
  ESCAPE_BACK,           // Ưu tiên lùi
  ESCAPE_TURN,           // Ưu tiên xoay
  ESCAPE_SWERVE,         // Trượt ngang
  ESCAPE_NONE            // Không có hướng thoát
};

/* ==================== CONTEXT ==================== */
struct OaContext {
  OaFsmState state = OA_IDLE;
  uint32_t   stateT0 = 0;
  
  // Thông tin trước khi né
  float      headingBefore = 0.f;    // Heading ban đầu
  float      cruiseHeading = 0.f;    // Heading cruise (để quay về sau OA)
  float      targetHeading = 0.f;    // Heading mục tiêu cần xoay đến
  int16_t    frontDistBefore = 0;    // Khoảng cách trước ban đầu
  
  // Đánh giá 4 hướng
  int16_t    evalFront = 0;
  int16_t    evalBack = 0;
  int16_t    evalLeft = 0;
  int16_t    evalRight = 0;
  EscapePriority escapePlan = ESCAPE_NONE;
  
  // Thông số di chuyển
  float      poseXBefore = 0.f;
  float      poseYBefore = 0.f;
  float      distBackup = 0.f;        // Quãng lùi đã đi
  float      distTurn = 0.f;         // Quãng xoay đã đi
  uint8_t    attempts = 0;
  int8_t     swerveDir = 0;          // Hướng trượt: 1=phải, -1=trái
  bool       dynamicCheckDone = false;
};

/** Global OA context (defined in SuperMarketBot-IOT.ino). */
extern OaContext g_oaCtx;

/* ==================== HELPERS ==================== */
static inline float oaNorm(float a) {
  while (a >  (float)M_PI) a -= 2.f * (float)M_PI;
  while (a < -(float)M_PI) a += 2.f * (float)M_PI;
  return a;
}

static inline float oaAngleDiff(float target, float current) {
  return oaNorm(target - current);
}

static inline uint16_t oaPct2Pwm(uint8_t pct) {
  return (uint16_t)((uint32_t)PWM_MAX * (uint32_t)pct / 100u);
}

static inline const char* oaStateName(OaFsmState s) {
  switch (s) {
    case OA_IDLE:           return "IDLE";
    case OA_EVALUATE:       return "EVALUATE";
    case OA_WAIT_DYNAMIC:   return "WAIT_DYN";
    case OA_STRAIGHTEN:     return "STRAIGHTEN";
    case OA_BACKUP:         return "BACKUP";
    case OA_TURN_TO_CLEAR:  return "TURN_CLEAR";
    case OA_SWERVE_PASS:    return "SWERVE";
    case OA_RESUME_CRUISE:  return "RESUME";
    case OA_BLOCKED:        return "BLOCKED";
    default:                return "?";
  }
}

#define OA_LOG(from, to, ...) do { \
    Serial.printf("[OA t=%lums %s→%s] ", (unsigned long)millis(), \
                  oaStateName(from), oaStateName(to)); \
    Serial.printf(__VA_ARGS__); \
    Serial.println(); \
  } while(0)

#define OA_LOG_STAY(state, ...) do { \
    Serial.printf("[OA t=%lums %s] ", (unsigned long)millis(), oaStateName(state)); \
    Serial.printf(__VA_ARGS__); \
    Serial.println(); \
  } while(0)

static inline float oaDistMoved(const OaContext &ctx) {
  float dx = g_pose.x - ctx.poseXBefore;
  float dy = g_pose.y - ctx.poseYBefore;
  return sqrtf(dx * dx + dy * dy);
}

/* ==================== THRESHOLDS ==================== */
#ifndef SAFE_SIDE_CM
#define SAFE_SIDE_CM       20      // Khoảng cách an toàn 2 bên (hành lang hẹp dùng 20cm)
#endif

#ifndef SAFE_BACK_CM
#define SAFE_BACK_CM        30      // Khoảng cách an toàn phía sau
#endif

#ifndef MIN_ESCAPE_DIST_CM
#define MIN_ESCAPE_DIST_CM  40      // Khoảng cách tối thiểu để coi là thoát
#endif

#ifndef DYNAMIC_WAIT_MS
#define DYNAMIC_WAIT_MS     1500    // Chờ người di chuyển 1.5s
#endif

#ifndef MAX_BACKUP_M
#define MAX_BACKUP_M        0.3f    // Lùi tối đa 30cm
#endif

#ifndef BACKUP_SPEED_PCT
#define BACKUP_SPEED_PCT    45      // Tốc độ lùi %
#endif

#ifndef TURN_SPEED_PCT
#define TURN_SPEED_PCT      50      // Tốc độ xoay %
#endif

/* ==================== EVALUATE 4 DIRECTIONS ==================== */
/**
 * Đánh giá 4 hướng để quyết định hướng thoát
 * Trả về EscapePriority
 */
inline EscapePriority oaEvaluateDirections(OaContext &ctx) {
  ctx.evalFront = g_state.usFront;
  ctx.evalBack = g_state.usBack;
  ctx.evalLeft = g_state.usLeft;
  ctx.evalRight = g_state.usRight;
  
  // Kiểm tra hướng nào thông thoáng
  bool frontClear = obsPathClear(ctx.evalFront);
  bool backClear = obsCmValid(ctx.evalBack) && ctx.evalBack >= SAFE_BACK_CM;
  bool leftClear = obsCmValid(ctx.evalLeft) && ctx.evalLeft >= SAFE_SIDE_CM;
  bool rightClear = obsCmValid(ctx.evalRight) && ctx.evalRight >= SAFE_SIDE_CM;
  
  Serial.printf("[OA-EVAL] F:%dcm B:%dcm L:%dcm R:%dcm | FrontClear:%d BackClear:%d LeftClear:%d RightClear:%d\n",
    ctx.evalFront, ctx.evalBack, ctx.evalLeft, ctx.evalRight,
    frontClear, backClear, leftClear, rightClear);
  
  // Ưu tiên 1: Đường thẳng phía trước đã thông (vật cản động đã di chuyển)
  if (frontClear) {
    return ESCAPE_STRAIGHT;
  }
  
  // Ưu tiên 2: Lùi + xoay (nếu đường sau thông thoáng)
  if (backClear) {
    return ESCAPE_BACK;
  }
  
  // Ưu tiên 3: Một bên thông thoáng - trượt ngang
  if (leftClear && rightClear) {
    // Cả 2 bên đều thông - chọn bên rộng hơn
    if (ctx.evalLeft >= ctx.evalRight) {
      ctx.swerveDir = -1; // Trái rộng hơn, trượt sang trái
    } else {
      ctx.swerveDir = 1;  // Phải rộng hơn, trượt sang phải
    }
    return ESCAPE_SWERVE;
  }
  if (leftClear) {
    ctx.swerveDir = -1;
    return ESCAPE_SWERVE;
  }
  if (rightClear) {
    ctx.swerveDir = 1;
    return ESCAPE_SWERVE;
  }
  
  // Ưu tiên 4: Xoay tại chỗ nếu có chỗ xoay
  // (ít nhất 1 trong 4 hướng có khoảng cách > 0)
  if (ctx.evalFront > 0 || ctx.evalBack > 0 || ctx.evalLeft > 0 || ctx.evalRight > 0) {
    return ESCAPE_TURN;
  }
  
  return ESCAPE_NONE;
}

/* ==================== RESET ==================== */
inline void oaReset(OaContext &ctx) {
  ctx.state = OA_IDLE;
  ctx.attempts = 0;
  ctx.dynamicCheckDone = false;
  ctx.distBackup = 0.f;
  ctx.distTurn = 0.f;
  ctx.swerveDir = 0;
}

/* ==================== BEGIN ==================== */
/**
 * Bắt đầu né vật cản
 */
inline bool oaBegin(OaContext &ctx, int16_t frontCm, uint32_t now) {
  if (!obsOaTriggered(frontCm)) {
    return false;
  }
  
  botStop();
  pidSpeedReset();
  pidYawReset();
  
  ctx.attempts++;
  ctx.headingBefore = g_pose.headingRad;
  ctx.frontDistBefore = frontCm;
  ctx.stateT0 = now;
  ctx.dynamicCheckDone = false;
  ctx.distBackup = 0.f;
  ctx.distTurn = 0.f;

  // Giới hạn số lần thử — tránh loop vô hạn
  if (ctx.attempts > 6) {
    Serial.printf("[OA] ABORT: %u lan thu, khong thoat duoc. Tra ve DONE.\n", (unsigned)ctx.attempts);
    ctx.state = OA_IDLE;
    ctx.attempts = 0;
    botStop();
    return false;  // Báo WP tiếp tục (bỏ qua obstacle)
  }

  // Chuyển sang đánh giá
  OaFsmState prev = ctx.state;
  ctx.state = OA_EVALUATE;
  OA_LOG(prev, OA_EVALUATE, "Bat dau danh gia 4 huong. VAT CAN %dcm. Lan thu %u", 
         frontCm, (unsigned)ctx.attempts);
  
  return true;
}

/* ==================== TICK ==================== */
inline OaTickResult oaTick(OaContext &ctx, int16_t frontCm, uint32_t now) {
  switch (ctx.state) {
    
  /* ==================== IDLE ==================== */
  case OA_IDLE:
    return OA_RES_IDLE;
    
  /* ==================== EVALUATE ==================== */
  case OA_EVALUATE: {
    // Đánh giá 4 hướng
    EscapePriority plan = oaEvaluateDirections(ctx);
    ctx.escapePlan = plan;
    
    OaFsmState prev = ctx.state;
    
    switch (plan) {
      case ESCAPE_STRAIGHT:
        // Đường trước đã thông - quay lại cruise
        OA_LOG(OA_EVALUATE, OA_IDLE, "Duong truoc da thong! Resume cruise");
        ctx.state = OA_IDLE;
        ctx.attempts = 0;
        return OA_RES_DONE;
        
      case ESCAPE_BACK: {
        // Lùi + xoay sang hướng thoát (left/right bên nào rộng hơn)
        // KHÔNG dùng headingBefore (sẽ chạy lại vào vật cản)
        float escapeOffset = (ctx.evalLeft >= ctx.evalRight) ? -1.57f : 1.57f; // 90° sang bên rộng
        ctx.targetHeading = oaNorm(ctx.headingBefore + escapeOffset);
        ctx.state = OA_BACKUP;
        ctx.stateT0 = now;
        ctx.poseXBefore = g_pose.x;
        ctx.poseYBefore = g_pose.y;
        OA_LOG(prev, OA_BACKUP, "Chon LUI+QUAY. Muc tieu: quay ve heading %.1fdeg",
               ctx.targetHeading * 180.0f / (float)M_PI);
        return OA_RES_RUNNING;
      }

      case ESCAPE_SWERVE: {
        // Với 4WD vi sai: xoay 90° sang bên trống, sau đó WP tự drive về target
        // swerveDir: +1=phải → xoay CW (-90° ← heading tăng = sang phải)
        //            -1=trái → xoay CCW (+90° ← heading giảm = sang trái)
        // Đã xác nhận: CW tăng heading, CCW giảm heading
        float turnOffset = (ctx.swerveDir > 0) ? (float)M_PI * 0.5f : -(float)M_PI * 0.5f;
        ctx.targetHeading = oaNorm(ctx.headingBefore + turnOffset);
        ctx.state = OA_SWERVE_PASS;
        ctx.stateT0 = now;
        ctx.poseXBefore = g_pose.x;
        ctx.poseYBefore = g_pose.y;
        OA_LOG(prev, OA_SWERVE_PASS, "Chon SWERVE sang %s. Xoay den %.1fdeg",
               ctx.swerveDir > 0 ? "PHAI" : "TRAI",
               ctx.targetHeading * 180.0f / (float)M_PI);
        return OA_RES_RUNNING;
      }

      case ESCAPE_TURN:
        // Xoay tại chỗ đến hướng trống nhất
        ctx.targetHeading = ctx.headingBefore;
        // Tính hướng xoay: nếu bên trái rộng hơn thì xoay sang trái
        if (ctx.evalLeft > ctx.evalRight) {
          ctx.swerveDir = -1;
        } else {
          ctx.swerveDir = 1;
        }
        ctx.state = OA_TURN_TO_CLEAR;
        ctx.stateT0 = now;
        OA_LOG(prev, OA_TURN_TO_CLEAR, "Chon XOAY %s de thoat",
               ctx.swerveDir > 0 ? "SANG PHAI" : "SANG TRAI");
        return OA_RES_RUNNING;
        
      default:
        // Không có hướng thoát
        ctx.state = OA_BLOCKED;
        ctx.stateT0 = now;
        OA_LOG(prev, OA_BLOCKED, "KHONG CO HUONG THOAT!");
        return OA_RES_BLOCKED;
    }
  }
  
  /* ==================== WAIT DYNAMIC ==================== */
  case OA_WAIT_DYNAMIC: {
    botStop();
    
    // Kiểm tra vật cản động đã di chuyển chưa
    if (obsPathClear(frontCm)) {
      OA_LOG(OA_WAIT_DYNAMIC, OA_IDLE, "Vat can dong da di chuyen. Resume!");
      ctx.state = OA_IDLE;
      ctx.attempts = 0;
      return OA_RES_DONE;
    }
    
    // Hết thời gian chờ
    if (now - ctx.stateT0 >= DYNAMIC_WAIT_MS) {
      // Quay lại đánh giá để quyết định hướng thoát
      ctx.state = OA_EVALUATE;
      ctx.stateT0 = now;
      OA_LOG(OA_WAIT_DYNAMIC, OA_EVALUATE, "Het cho, danh gia lai");
    }
    return OA_RES_RUNNING;
  }
  
  /* ==================== STRAIGHTEN ==================== */
  case OA_STRAIGHTEN: {
    // Quay về heading mục tiêu. BẮT BUỘC chờ tối thiểu 800ms để motor kịp xoay.
    float yawError = oaAngleDiff(ctx.targetHeading, g_pose.headingRad);
    uint32_t elapsed = now - ctx.stateT0;

    // Chờ tối thiểu 800ms VÀ lệch < 10° mới coi là done
    if (elapsed >= 800u && fabsf(yawError) < 0.175f) { // ~10°
      botStop();
      pidYawReset();
      OA_LOG(OA_STRAIGHTEN, OA_IDLE, "Da quay ve heading muc tieu! Resume cruise");
      ctx.state = OA_IDLE;
      ctx.attempts = 0;  // Reset chỉ khi thành công
      return OA_RES_DONE;
    }

    // Timeout STRAIGHTEN 4s → tiếp tục (GIẮ attempts để hard-limit hoạt động)
    if (elapsed >= 4000u) {
      botStop();
      OA_LOG(OA_STRAIGHTEN, OA_IDLE, "STRAIGHTEN timeout 4s. Attempts=%u", (unsigned)ctx.attempts);
      ctx.state = OA_IDLE;
      // KHÔNG reset attempts — để counter tăng dần đến hard-limit
      return OA_RES_DONE;
    }

    // Xoay về hướng mục tiêu
    // Đã xác nhận từ log: CW immediate → heading TĂNG, CCW immediate → heading GIẢM
    // yawError = target - current: dương → cần TĂNG heading → CW
    //                               âm    → cần GIẢM heading → CCW
    uint16_t turnPwm = oaPct2Pwm(TURN_SPEED_PCT);
    if (yawError > 0.f) botRotateCWImmediate(turnPwm);   // target > current → CW
    else                botRotateCCWImmediate(turnPwm);  // target < current → CCW

    return OA_RES_RUNNING;
  }
  
  /* ==================== BACKUP ==================== */
  case OA_BACKUP: {
    // Lùi
    botBackward(oaPct2Pwm(BACKUP_SPEED_PCT));
    
    // Theo dõi khoảng cách đã lùi
    float dist = oaDistMoved(ctx);
    ctx.distBackup = dist;
    
    // Kiểm tra có vật cản phía sau không
    if (obsCmValid(ctx.evalBack) && ctx.evalBack < SAFE_BACK_CM) {
      botStop();
      OA_LOG_STAY(OA_BACKUP, "Phia sau co vat can! Dung lai");
      ctx.state = OA_TURN_TO_CLEAR;
      ctx.stateT0 = now;
      return OA_RES_RUNNING;
    }
    
    // Lùi đủ xa hoặc đường sau đã thông thoáng
    if (dist >= MAX_BACKUP_M || obsCmValid(ctx.evalBack) && ctx.evalBack >= SAFE_BACK_CM * 1.5f) {
      botStop();
      pidYawReset();
      
      // Xoay về heading ban đầu (phía trước)
      ctx.state = OA_STRAIGHTEN;
      ctx.stateT0 = now;
      OA_LOG(OA_BACKUP, OA_STRAIGHTEN, "Lui %.2fm, quay ve heading", dist);
      return OA_RES_RUNNING;
    }
    
    return OA_RES_RUNNING;
  }
  
  /* ==================== TURN TO CLEAR ==================== */
  case OA_TURN_TO_CLEAR: {
    // Xoay tại chỗ để hướng về phía trống
    float yawError = oaAngleDiff(ctx.targetHeading, g_pose.headingRad);
    float absError = fabsf(yawError);
    
    // Kiểm tra đã xoay đủ chưa
    if (absError < 0.15f) { // ~8.5 độ
      botStop();
      
      // Kiểm tra đường phía trước đã thông chưa
      if (obsPathClear(frontCm)) {
        ctx.state = OA_RESUME_CRUISE;
        ctx.stateT0 = now;
        OA_LOG(OA_TURN_TO_CLEAR, OA_RESUME_CRUISE, "Da xoay, duong truoc thong. Resume!");
      } else {
        // Vẫn chưa thông, thử trượt ngang
        EscapePriority plan = oaEvaluateDirections(ctx);
        if (plan == ESCAPE_SWERVE) {
          ctx.state = OA_SWERVE_PASS;
          ctx.stateT0 = now;
          OA_LOG(OA_TURN_TO_CLEAR, OA_SWERVE_PASS, "Duong truoc van chua thong, thu SWERVE");
        } else {
          // Xoay thêm 1 góc
          ctx.targetHeading = oaNorm(ctx.targetHeading + ctx.swerveDir * 0.5f);
          OA_LOG_STAY(OA_TURN_TO_CLEAR, "Van chua thong, xoay them");
        }
      }
      return OA_RES_RUNNING;
    }
    
    // Xoay
    if (ctx.swerveDir > 0) {
      botRotateCW(oaPct2Pwm(TURN_SPEED_PCT));
    } else {
      botRotateCCW(oaPct2Pwm(TURN_SPEED_PCT));
    }
    
    return OA_RES_RUNNING;
  }
  
  /* ==================== SWERVE PASS ==================== */
  /* Với bánh thường (4WD vi sai) KHÔNG thể trượt ngang.
   * Chiến lược: Xoay 90° về hướng trống → rồi để WP resume tự drive về target.
   * Dùng lại state STRAIGHTEN để xoay đến targetHeading đã tính sẵn.         */
  case OA_SWERVE_PASS: {
    // targetHeading đã được set ở EVALUATE (headingBefore ± 90°)
    // Chuyển ngay sang STRAIGHTEN để xoay đến đó
    ctx.stateT0 = now;
    ctx.state = OA_STRAIGHTEN;
    OA_LOG(OA_SWERVE_PASS, OA_STRAIGHTEN,
           "4WD: xoay %.0fdeg de thoat", ctx.targetHeading * 180.0f / (float)M_PI);
    return OA_RES_RUNNING;
  }

  /* ==================== RESUME CRUISE ==================== */
  case OA_RESUME_CRUISE: {
    // Không dùng nữa — STRAIGHTEN đã xử lý rồi DONE trực tiếp
    // Fallthrough về IDLE
    OA_LOG(OA_RESUME_CRUISE, OA_IDLE, "Resume cruise");
    ctx.state = OA_IDLE;
    ctx.attempts = 0;
    return OA_RES_DONE;
  }


  /* ==================== BLOCKED ==================== */
  case OA_BLOCKED: {
    botStop();
    
    // Chờ 2 giây rồi thử lại
    if (now - ctx.stateT0 > 2000) {
      ctx.state = OA_EVALUATE;
      ctx.stateT0 = now;
      OA_LOG(OA_BLOCKED, OA_EVALUATE, "Thu lai lan %u", (unsigned)ctx.attempts);
      return OA_RES_RUNNING;
    }
    
    return OA_RES_BLOCKED;
  }
  
  /* ==================== DEFAULT ==================== */
  default:
    ctx.state = OA_IDLE;
    return OA_RES_IDLE;
  }
}

/* ==================== CRUISE FORWARD ==================== */
/**
 * Đi thẳng có kiểm soát heading bằng IMU
 */
inline bool oaCruiseForward(OaContext &ctx, int16_t frontCm, uint16_t cruisePwm) {
  if (!obsPathClear(frontCm)) {
    ctx.state = OA_IDLE;
    return false;
  }
  
  // Kiểm tra 2 bên để né tường
  int16_t strafe = 0;
  if (obsCmValid(obsLeftCm()) && obsLeftCm() < SAFE_SIDE_CM * 2) {
    strafe += 50; // Trượt sang phải
  }
  if (obsCmValid(obsRightCm()) && obsRightCm() < SAFE_SIDE_CM * 2) {
    strafe -= 50; // Trượt sang trái
  }
  strafe = constrain(strafe, -80, 80);
  
  // PID cho tốc độ
  float dt_s = (float)SAFE_LOOP_MS * 0.001f;
  float pidOut = pidSpeedCompute(pwmToEstMps(cruisePwm), robotActualSpeedMps(), dt_s);
  int32_t run = (int32_t)cruisePwm + (int32_t)pidOut;
  run = constrain(run, 0, (int32_t)PWM_MAX);
  
  // Yaw control bằng IMU
  int16_t steer = 0;
#if USE_IMU_MPU6050
  extern bool g_imuEnabled;
  if (g_imuEnabled) {
    float yawOut = pidYawCompute(ctx.headingBefore, g_pose.headingRad, dt_s);
    steer = (int16_t)constrain(yawOut, -100, 100);
  }
#endif
  
  if (strafe != 0) {
    // Bánh thường: bỏ qua strafe (không có khả năng trượt ngang)
    botDrive(steer, 100, (uint16_t)run);
  } else {
    botDrive(steer, 100, (uint16_t)run);
  }
  
  return true;
}

#endif // LOCAL_OBSTACLE_AVOID_H
