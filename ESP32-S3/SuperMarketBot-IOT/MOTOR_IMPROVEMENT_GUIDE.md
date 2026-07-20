# CẢI TIẾN MOTOR CONTROL - HƯỚNG DẪN TÍCH HỢP

## TỔNG QUAN

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         CẢI TIẾN MOTOR CONTROL                             │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ✅ Giữ nguyên TẤT CẢ firmware cũ:                                      │
│     - WebServer, MQTT, WebUI, WaypointNav                                │
│     - YDLIDAR Driver (ESP32)                                            │
│     - Odometry, IMU, Sensors                                             │
│                                                                             │
│  ✅ CHỈ cải tiến:                                                         │
│     - MotorControlPro.h — Smoothing, Anti-jerk                            │
│     - PidControllerPro.h — Adaptive PID, Derivative Filter                  │
│                                                                             │
│  ✅ Kết quả: Robot di chuyển MƯỢT MÀ hơn, ít giật hơn                   │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## CÁC FILE ĐÃ TẠO

### 1. MotorControlPro.h
Cải tiến từ Motors.h:
- Smooth acceleration/deceleration
- Joystick input filtering
- Cubic velocity mapping
- Anti-jerk mechanism
- S-curve velocity profile

### 2. PidControllerPro.h
Cải tiến từ PidController.h:
- Adaptive PID gains
- Derivative low-pass filter
- Dead-zone cho error nhỏ
- Feedforward term
- Bang-bang pre-action
- Better anti-windup

---

## CÁCH TÍCH HỢP

### PHƯƠNG ÁN 1: THAY THẾ TRỰC TIẾP (ĐƠN GIẢN NHẤT)

**Bước 1: Backup file cũ**
```bash
copy Motors.h Motors.h.bak
copy PidController.h PidController.h.bak
```

**Bước 2: Copy file mới**
```bash
copy MotorControlPro.h Motors.h
copy PidControllerPro.h PidController.h
```

**Bước 3: Upload firmware và test**

**⚠️ LƯU Ý:** Cách này có thể gây **conflict** nếu có dependency.

---

### PHƯƠNG ÁN 2: INCLUDE CÙNG LÚC (KHUYẾN NGHỊ)

**Bước 1: Giữ nguyên file cũ**

**Bước 2: Thêm include vào SuperMarketBot-IOT.ino**

Tìm phần include trong file `.ino`:

```cpp
// Thêm vào đầu file (sau các include cũ)
#include "Motors.h"
#include "PidController.h"

// THÊM DÒNG NÀY:
#include "MotorControlPro.h"   // Cải tiến motor control
#include "PidControllerPro.h"  // Cải tiến PID
```

**Bước 3: Sử dụng hàm mới**

Trong `SuperMarketBot-IOT.ino`, tìm và thay:

```cpp
// THAY:
// botDriveMecanum(g_state.cmdStrafe, g_state.cmdY, g_state.cmdX, g_state.baseSpeed);

// THÀNH:
// botDriveMecanumPro(g_state.cmdStrafe, g_state.cmdY, g_state.cmdX, g_state.baseSpeed, true);
```

---

### PHƯƠNG ÁN 3: SỬA ĐỔI CÓ CHỌN LỌC (AN TOÀN NHẤT)

**Bước 1: Chỉ cần sửa 2 file nhỏ**

#### File 1: Motors.h

Thêm vào cuối file (trước `#endif`):

```cpp
// ========== CẢI TIẾN SMOOTH ==========

/** Tốc độ thay đổi PWM tối đa mỗi tick (50ms) */
#define MOTOR_SMOOTH_RAMP_MAX  80

/** Low-pass filter cho joystick */
#define MOTOR_JOYSTICK_FILTER_ALPHA 0.7f

/** Smooth motor drive với acceleration limiting */
inline void motorDriveSmooth(MotorId id, int32_t speed) {
    static int32_t lastSpeed[4] = {0, 0, 0, 0};
    uint8_t idx = (uint8_t)id;
    
    int32_t delta = speed - lastSpeed[idx];
    if (delta > MOTOR_SMOOTH_RAMP_MAX) delta = MOTOR_SMOOTH_RAMP_MAX;
    if (delta < -MOTOR_SMOOTH_RAMP_MAX) delta = -MOTOR_SMOOTH_RAMP_MAX;
    
    lastSpeed[idx] += delta;
    motorDrive(id, lastSpeed[idx]);
}

/** Cải tiến botDriveMecanum */
inline void botDriveMecanumSmooth(int16_t strafe, int16_t fwd, int16_t turn, uint16_t base) {
    // Sử dụng cubic curve thay vì quadratic
    int32_t fwdSign = (fwd >= 0) ? 1 : -1;
    int32_t strafeSign = (strafe >= 0) ? 1 : -1;
    int32_t turnSign = (turn >= 0) ? 1 : -1;

    // Cubic mapping (mượt hơn quadratic)
    int32_t fwdCurve = ((int32_t)fwd * (int32_t)fwd * (int32_t)fwd * fwdSign) / 10000;
    int32_t strafeCurve = ((int32_t)strafe * (int32_t)strafe * (int32_t)strafe * strafeSign) / 10000;
    int32_t turnCurve = ((int32_t)turn * (int32_t)turn * (int32_t)turn * turnSign) / 10000;

    int32_t fwdScaled = fwdCurve * (int32_t)base / 100;
    int32_t strafeScaled = strafeCurve * (int32_t)base / 100;
    int32_t turnScaled = turnCurve * (int32_t)base / 100;

    constexpr int32_t STRAFE_GAIN = 140;
    constexpr int32_t FWD_GAIN = 115;
    constexpr int32_t TURN_GAIN = 115;

    int32_t fl = (fwdScaled * FWD_GAIN + strafeScaled * STRAFE_GAIN + turnScaled * TURN_GAIN) / 100;
    int32_t rl = (fwdScaled * FWD_GAIN - strafeScaled * STRAFE_GAIN + turnScaled * TURN_GAIN) / 100;
    int32_t fr = (fwdScaled * FWD_GAIN - strafeScaled * STRAFE_GAIN - turnScaled * TURN_GAIN) / 100;
    int32_t rr = (fwdScaled * FWD_GAIN + strafeScaled * STRAFE_GAIN - turnScaled * TURN_GAIN) / 100;

    motorDriveSmooth(MID_FL, fl);
    motorDriveSmooth(MID_RL, rl);
    motorDriveSmooth(MID_FR, fr);
    motorDriveSmooth(MID_RR, rr);
}

// ========== KẾT THÚC CẢI TIẾN ==========

#endif // MOTORS_H
```

#### File 2: PidController.h

Thêm vào cuối file (trước `#endif`):

```cpp
// ========== CẢI TIẾN PID PRO ==========

#define PID_SPEED_KP_PRO    8.0f
#define PID_SPEED_KI_PRO    0.5f
#define PID_SPEED_KD_PRO    1.5f
#define PID_YAW_KP_PRO      25.0f
#define PID_YAW_KI_PRO      0.5f
#define PID_YAW_KD_PRO      3.0f
#define PID_DERIV_FILTER    0.3f

static PidState s_pidSpeedPro = {
    PID_SPEED_KP_PRO, PID_SPEED_KI_PRO, PID_SPEED_KD_PRO,
    0.5f, (float)PWM_MAX, 0.f, 0.f, 0.f, true
};

static PidState s_pidYawPro = {
    PID_YAW_KP_PRO, PID_YAW_KI_PRO, PID_YAW_KD_PRO,
    10.0f, 100.0f, 0.f, 0.f, 0.f, true
};

inline float pidSpeedComputePro(float targetMps, float actualMps, float dt_s) {
    if (dt_s <= 0.f) return 0.f;
    
    float err = targetMps - actualMps;
    if (abs(err) < 0.01f) return 0.f; // Deadzone
    
    // Derivative with filter
    float deriv = 0.f;
    if (!s_pidSpeedPro.firstRun) {
        deriv = (err - s_pidSpeedPro.prevError) / dt_s;
        deriv = PID_DERIV_FILTER * deriv + (1.0f - PID_DERIV_FILTER) * s_pidSpeedPro.derivativeFilter;
        s_pidSpeedPro.derivativeFilter = deriv;
    }
    s_pidSpeedPro.firstRun = false;
    s_pidSpeedPro.prevError = err;
    
    // Integral with anti-windup
    s_pidSpeedPro.integral += err * dt_s;
    if (s_pidSpeedPro.integral > 0.5f) s_pidSpeedPro.integral = 0.5f;
    if (s_pidSpeedPro.integral < -0.5f) s_pidSpeedPro.integral = -0.5f;
    
    float out = s_pidSpeedPro.kp * err + s_pidSpeedPro.ki * s_pidSpeedPro.integral + s_pidSpeedPro.kd * deriv;
    return constrain(out, -PWM_MAX, PWM_MAX);
}

inline float pidYawComputePro(float targetRad, float actualRad, float dt_s) {
    if (dt_s <= 0.f) return 0.f;
    
    float err = targetRad - actualRad;
    while (err > M_PI) err -= 2.0f * M_PI;
    while (err < -M_PI) err += 2.0f * M_PI;
    
    if (abs(err) < 0.02f) return 0.f; // Deadzone
    
    // Pre-action for large errors
    if (abs(err) > 0.15f) {
        s_pidYawPro.integral *= 0.5f;
        return (err > 0) ? 80.0f : -80.0f;
    }
    
    // Derivative with filter
    float deriv = 0.f;
    if (!s_pidYawPro.firstRun) {
        deriv = (err - s_pidYawPro.prevError) / dt_s;
        deriv = PID_DERIV_FILTER * deriv + (1.0f - PID_DERIV_FILTER) * s_pidYawPro.derivativeFilter;
        s_pidYawPro.derivativeFilter = deriv;
    }
    s_pidYawPro.firstRun = false;
    s_pidYawPro.prevError = err;
    
    // Integral with anti-windup
    s_pidYawPro.integral += err * dt_s;
    if (s_pidYawPro.integral > 10.0f) s_pidYawPro.integral = 10.0f;
    if (s_pidYawPro.integral < -10.0f) s_pidYawPro.integral = -10.0f;
    
    float out = s_pidYawPro.kp * err + s_pidYawPro.ki * s_pidYawPro.integral + s_pidYawPro.kd * deriv;
    return constrain(out, -100.0f, 100.0f);
}

inline void pidSpeedResetPro() {
    s_pidSpeedPro.integral = 0.f;
    s_pidSpeedPro.prevError = 0.f;
    s_pidSpeedPro.derivativeFilter = 0.f;
    s_pidSpeedPro.firstRun = true;
}

inline void pidYawResetPro() {
    s_pidYawPro.integral = 0.f;
    s_pidYawPro.prevError = 0.f;
    s_pidYawPro.derivativeFilter = 0.f;
    s_pidYawPro.firstRun = true;
}

// ========== KẾT THÚC CẢI TIẾN PID PRO ==========

#endif // PID_CONTROLLER_H
```

**Bước 3: Update SuperMarketBot-IOT.ino**

Trong taskControl, tìm:

```cpp
botDriveMecanum(g_state.cmdStrafe, g_state.cmdY, g_state.cmdX, g_state.baseSpeed);
```

Thay thành:

```cpp
botDriveMecanumSmooth(g_state.cmdStrafe, g_state.cmdY, g_state.cmdX, g_state.baseSpeed);
```

---

## SO SÁNH TRƯỚC VÀ SAU CẢI TIẾN

### Motor Control

| Thông số | TRƯỚC | SAU |
|-----------|--------|-----|
| Acceleration | Tức thì (có thể giật) | Smooth ramp (80 PWM/tick) |
| Joystick mapping | Quadratic | Cubic (mượt hơn) |
| Strafe gain | 135 | 140 (nhạy hơn) |
| Jerk | Cao | Thấp (có limiting) |

### PID Control

| Thông số | TRƯỚC | SAU |
|-----------|--------|-----|
| Speed Kp | 0 (P-only) | 8.0 (PI-D) |
| Speed Ki | 0 | 0.5 |
| Speed Kd | 0 | 1.5 |
| Yaw Kp | 15 | 25 |
| Derivative filter | Không | Có (0.3 alpha) |
| Deadzone | Không | Có (chống hunting) |
| Pre-action | Không | Có (bang-bang) |

---

## KẾT QUẢ KỲ VỌNG

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         KẾT QUẢ CẢI TIẾN                                   │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ✅ Robot KHỞI HÀNH mượt hơn (smooth acceleration)                        │
│  ✅ Robot DỪNG mượt hơn (smooth deceleration)                              │
│  ✅ Ít GIẬT khi chuyển hướng (jerk limiting)                              │
│  ✅ Điều khiển joystick NHẠY hơn ở tốc độ thấp (cubic curve)           │
│  ✅ Ít OSCILLATION khi đi thẳng (derivative filter)                      │
│  ✅ KHÔNG overshoot khi đến waypoint (anti-windup)                        │
│  ✅ Phản hồi NHANH khi lệch hướng (pre-action + adaptive gain)            │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## NẾU CẦN QUAY LẠI

```bash
# Copy backup
copy Motors.h.bak Motors.h
copy PidController.h.bak PidController.h

# Hoặc xóa các thay đổi trong .ino
```

---

## LƯU Ý QUAN TRỌNG

1. **YDLIDAR/SLAM vẫn ở ESP32** — bạn có thể từ từ di chuyển sang Android sau
2. **WebServer, MQTT, WebUI vẫn hoạt động** — không thay đổi gì
3. **Waypoint Navigation vẫn giữ nguyên** — chỉ cải thiện cách robot THỰC HIỆN lệnh
4. **Backend vẫn giao tiếp bình thường** — protocol không đổi

---

## NEXT STEPS

1. **Test cơ bản**: Upload firmware → Test joystick
2. **So sánh**: Trước và sau cải tiến
3. **Fine-tune**: Điều chỉnh PID gains nếu cần
4. **YDLIDAR/SLAM**: Từ từ di chuyển sang Android Native Hub

---

**Ngày cập nhật**: 20/07/2026
**Phiên bản**: 1.0
