/* =====================================================================
 *  Config.h — SmartMarketBot Mini 4WD
 *  Khai báo toàn bộ chân GPIO, hằng số hệ thống và tham số hiệu chỉnh
 *  Board: ESP32-S3-DevKitC N16R8 (16MB Flash / 8MB Octal PSRAM)
 *  Ghi chú: Tránh GPIO 19,20 (USB Native) và 33-37 (Octal PSRAM)
 * =====================================================================*/
#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

/* ---------------------- ĐỘNG LỰC (2x TB6612FNG) --------------------- */
// Mạch 1 — Bên TRÁI
#define M_FL_PWM      4     // Front-Left  PWMA
#define M_FL_IN1      5     // Front-Left  AIN1
#define M_FL_IN2      6     // Front-Left  AIN2

#define M_RL_PWM      7     // Rear-Left   PWMB
#define M_RL_IN1      8     // Rear-Left   BIN1
#define M_RL_IN2      9     // Rear-Left   BIN2

// Mạch 2 — Bên PHẢI
#define M_FR_PWM      21    // Front-Right PWMA
#define M_FR_IN1      38    // Front-Right AIN1
#define M_FR_IN2      39    // Front-Right AIN2

#define M_RR_PWM      40    // Rear-Right  PWMB
#define M_RR_IN1      41    // Rear-Right  BIN1
#define M_RR_IN2      42    // Rear-Right  BIN2

#define M_STBY        47    // Standby chung cho 2 TB6612

/* -------------------- LIDAR TF-Luna (UART) -------------------------- */
// LiDAR trước dùng Serial1
#define LIDAR_F_TX    17    // TX ESP32 → RX LiDAR (không bắt buộc nối)
#define LIDAR_F_RX    18    // RX ESP32 ← TX LiDAR

// LiDAR sau dùng Serial2
#define LIDAR_B_TX    1
#define LIDAR_B_RX    2

#define LIDAR_BAUD    115200
// Góc đo TF-Luna thường 2,3°/step; tầm datasheet ~0,2m–8m (phụ thuộc vật liệu, ánh sáng)
#define LIDAR_MAX_CM  800

/* -------------------- SIÊU ÂM (4x HC-SR04) ------------------------- */
#define US_TRIG       14    // Trigger dùng chung
#define US_ECHO_F     10    // Echo trước
#define US_ECHO_B     11    // Echo sau
#define US_ECHO_L     12    // Echo trái
#define US_ECHO_R     13    // Echo phải
// NewPing: tham số tối đa (ms chờ) ~ tương ứng ~2m — đủ thực tế, ping không quá lâu
#define US_PING_MAX_CM  200
// Trong hành lang / siêu thị: HC-SR04 ổn định thường ~1,2–1,8m; dùng 1,6m cho HMI + coi như "xa"
#define US_DISPLAY_MAX_CM 160

/* -------------------- ENCODER (4x FC-03) --------------------------- */
#define ENC_FL        15    // Front-Left
#define ENC_RL        16    // Rear-Left
#define ENC_FR        3     // Front-Right   (strapping pin JTAG, OK input)
#define ENC_RR        48    // Rear-Right    (lưu ý: là LED RGB trên 1 số board)

// Số xung trên 1 vòng bánh xe (tuỳ đĩa encoder - thường 20 khe chữ U)
#define ENC_PPR       20.0f
// Chu vi bánh xe (mét) để tính quãng đường — ví dụ bánh D=65mm
#define WHEEL_DIAM_M  0.065f
#define WHEEL_CIRC_M  (PI * WHEEL_DIAM_M)

/* -------------------- PWM / LEDC ----------------------------------- */
#define PWM_FREQ      20000 // 20kHz, ngoài ngưỡng nghe
#define PWM_RES_BITS  10    // 0..1023
#define PWM_MAX       ((1 << PWM_RES_BITS) - 1)

/* -------------------- AN TOÀN NÉ VẬT CẢN (không gian mở: siêu thị / hành lang) - */
// Dừng khẩn cấp: cận tường/đùi người (LiDAR + siêu âm) — 20cm an toàn hơn 15 khi tốc độ tăng
#define SAFE_STOP_CM    20
// Giảm tốc từ từ theo tầm LiDAR/ US khi tới < 2m (phù hợp tầm 8m)
#define SAFE_SLOW_CM    200
#define SAFE_LOOP_MS    30    // Chu kỳ vòng an toàn (ms)

/* -------------------- WIFI SOFTAP ---------------------------------- */
#define AP_SSID       "SmartMarketBot"
#define AP_PASS       "12345678"
#define WEB_PORT      80
#define WS_PORT       81

/* -------------------- LED RGB onboard (WS2812, thường GPIO 48 trên DevKitC) -- */
// Trùng với ENC_RR nếu cùng GPIO 48: bật LED = tắt đếm encoder bánh sau phải (ISR).
// Để dùng đủ 4 encoder, đặt SMB_ONBOARD_RGB = 0.
#define SMB_ONBOARD_RGB     1
#define SMB_NEOPIXEL_PIN    48
#define SMB_NEOPIXEL_COUNT  1
#define SMB_RGB_BRIGHTNESS  40  // 0–255, giảm nếu chói mắt

/* -------------------- LƯU TRỮ -------------------------------------- */
#define NVS_NAMESPACE "smb"

/* -------------------- CHẾ ĐỘ HOẠT ĐỘNG ----------------------------- */
enum RobotMode : uint8_t {
  MODE_MANUAL = 0,    // Lái tay
  MODE_AUTO   = 1     // Tự hành né vật cản
};

/* -------------------- CẤU TRÚC CHIA SẺ GIỮA 2 CORE ----------------- */
struct RobotState {
  // Cảm biến khoảng cách (cm)
  volatile int16_t usFront, usBack, usLeft, usRight;
  volatile int16_t lidarFront, lidarBack;
  // Tốc độ bánh xe (RPM)
  volatile float rpmFL, rpmRL, rpmFR, rpmRR;
  // Quãng đường (m) ước lượng
  volatile float distFL, distRL, distFR, distRR;
  // Điều khiển
  volatile int16_t cmdX;      // -100..100 (trái/phải)
  volatile int16_t cmdY;      // -100..100 (tiến/lùi)
  volatile uint16_t baseSpeed; // 0..PWM_MAX tốc độ nền
  volatile RobotMode mode;
  volatile bool estop;        // Cờ dừng khẩn cấp
};

extern RobotState g_state;

#endif // CONFIG_H
