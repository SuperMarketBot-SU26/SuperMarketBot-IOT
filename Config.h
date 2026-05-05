/* =====================================================================
 *  Config.h — SmartMarketBot Mini 4WD
 *  Board: ESP32-S3-DevKitC-1 (N16R8) — theo sơ đồ Espressif
 *  Tránh: GPIO 19,20 (USB D+/D-), 33-37 (Octal PSRAM trên module N16R8)
 *  Phải tránh: GPIO 38 = RGB LED nội bộ — KHÔNG dùng cho TB6612
 * =====================================================================*/
#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

/* ---------------------- ĐỘNG LỰC (2x TB6612FNG) --------------------- */
// Mạch 1 — Bên TRÁI (chân 4~9, TB6612 #1: Motor FL + RL)
#define M_FL_PWM      4     // Front-Left  PWMA  (LEDC)
#define M_FL_IN1      5     // Front-Left  AIN1
#define M_FL_IN2      6     // Front-Left  AIN2

#define M_RL_PWM      7     // Rear-Left   PWMB
#define M_RL_IN1      8     // Rear-Left   BIN1
#define M_RL_IN2      9     // Rear-Left   BIN2

// Mạch 2 — Bên PHẢI (TB6612 #2: Motor FR + RR) — 38,39 bị chiếm bởi LED/JTAG, chuyển 45,46
#define M_FR_PWM      21    // Front-Right PWMA  (LEDC)
#define M_FR_IN1      45    // Front-Right AIN1  (từ 38, đúng theo DevKitC: 38=RGB)
#define M_FR_IN2      46    // Front-Right AIN2  (từ 39)

#define M_RR_PWM      40    // Rear-Right  PWMB
#define M_RR_IN1      41    // Rear-Right  BIN1
#define M_RR_IN2      42    // Rear-Right  BIN2

#define M_STBY        47    // Standby chung cho 2 TB6612

/* -------------------- LIDAR TF-Luna (UART) -------------------------- */
// Cặp 17/18 = U1 theo sơ đồ board; LiDAR: 5V, GND, TX/Luna → RX ESP, RX/Luna ← TX ESP
#define LIDAR_F_TX    17
#define LIDAR_F_RX    18
// Cặp 1/2: Serial2 — tránh 19,20; đủ cho UART 2
#define LIDAR_B_TX    1
#define LIDAR_B_RX    2

#define LIDAR_BAUD    115200
// Góc đo TF-Luna thường 2,3°/step; tầm datasheet ~0,2m–8m (phụ thuộc vật liệu, ánh sáng)
#define LIDAR_MAX_CM  800
/**
 * TF-Luna (Benewake): gửi lệnh UART sau khi mở cổng — bật output, khung 9 byte (cm), FPS, save.
 * Tắt (=0) nếu bạn đã cấu hình bằng tool PC và không muốn firmware đụng vào.
 */
#define TFLUNA_SEND_INIT_CMD  1
/** Tần số mẫu 1–250 Hz (chỉ khi TFLUNA_SEND_INIT_CMD=1). */
#define TFLUNA_SAMPLE_HZ      100

/* -------------------- SIÊU ÂM (4x HC-SR04) ------------------------- */
// VCC 5V, GND; Trig 3,3V OK; Echo 5V → nên 1k+2k chia áp 3,3V vào chân dưới
#define US_TRIG       14    // Trigger dùng chung (1 chân)
#define US_ECHO_F     10
#define US_ECHO_B     11
#define US_ECHO_L     12
#define US_ECHO_R     13
// NewPing: tham số tối đa (ms chờ) ~ tương ứng ~2m — đủ thực tế, ping không quá lâu
#define US_PING_MAX_CM  200
// Trong hành lang / siêu thị: HC-SR04 ổn định thường ~1,2–1,8m; dùng 1,6m cho HMI + coi như "xa"
#define US_DISPLAY_MAX_CM 160

/* -------------------- ENCODER (cảm biến gạt/MH, DO nối ESP) ------- */
// DO → GPIO + interrupt; 3,3/5V theo lô module (thường 3,3V OK)
// GPIO3 (ENC_FR): trên S3 là MTCK/JTAG — trong Arduino chọn USB JTAG disabled / peripheral JTAG off nếu encoder không đếm xung.
#define ENC_FL        15    // Trước trái
#define ENC_RL        16    // Sau trái
#define ENC_FR        3     // Trước phải (JTAG, chỉ dùng input)
#define ENC_RR        48    // Sau phải (DevKitC-1: LED RGB = 38, không dùng 38 cho enc)

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
// Vùng "dừng cứng" (cm): dùng trong tự lái + né tránh. Tăng 26–35 nếu nhiễu/dừng sớm quá.
#define SAFE_STOP_CM    28
// Bắt đầu giảm tốc khi vật trước gần hơn ngưỡng (cm). 80–120 = ít nhạy từ xa; 180–220 = nhả ga sớm.
#define SAFE_SLOW_CM    100
#define SAFE_LOOP_MS    30    // Chu kỳ vòng an toàn (ms)
/** Ngưỡng HC-SR04 trái/phải (cm) để bẻ lái trong AUTO — nhỏ hơn SAFE_STOP = ít "giật" ngang. */
#define SAFE_SIDE_AVOID_CM  14

/* -------------------- WIFI SOFTAP ---------------------------------- */
#define AP_SSID         "SmartMarketBot"
#define AP_PASS         "12345678"
/** Kênh 2.4 GHz (1–11). 6 thường ít chồng lấn; tránh kênh “lạ” nếu điện thoại lọc theo vùng. */
#define AP_WIFI_CHANNEL 6
#define AP_MAX_CLIENTS  4
#define WEB_PORT        80
#define WS_PORT         81

/* -------------------- ĐO PIN (ADC, tùy chọn) ------------------------
 *  ESP chỉ đọc được 0..~3.3 V trên chân ADC — cần chiết áp 2 điện trở từ nguồn
 *  muốn theo dõi (khuyến nghị: điểm 12 V trước / sau pin, GND chung với ESP).
 *  Không nên chỉ đo 5 V sau XL4015 để suy % pin: buck ổn định 5 V trong khi 12 V
 *  vẫn sụt — % trên web sẽ không đúng.
 *
 *  Ví dụ: 12V → R1(68k) → chân ADC → R2(10k) → GND
 *         Vadc = Vbat * R2/(R1+R2)  (đặt BAT_DIV_* khớp R thật của bạn)
 *  Bật BAT_MONITOR_ENABLE 1 và đặt BAT_ADC_PIN trùng chân còn trống + hỗ trợ ADC. */
#define BAT_MONITOR_ENABLE  0
#define BAT_ADC_PIN         39
#define BAT_DIV_R1_KOHM     68.0f
#define BAT_DIV_R2_KOHM     10.0f
/** Ngưỡng V pin (tại điểm đo) — chỉnh theo loại pin (3S Li-ion, 12V SLA, …). */
#define BAT_V_FULL          12.6f
#define BAT_V_EMPTY         10.2f

/* -------------------- LED: chỉ RGB zin sẵn trên bo DevKit (WS2812, GPIO 38) --- */
// Không dùng thêm bóng / dải LED ngoài — ngoài linh kiện robot chỉ: 2 LiDAR, 4 US, 4 enc, 2 driver.
// Trùng GPIO: 38 dành cho LED tích hợp, ENC_RR=48 nên bật LED vẫn đủ 4 encoder.
#define SMB_ONBOARD_RGB     1
#define SMB_NEOPIXEL_PIN    38
#define SMB_NEOPIXEL_COUNT  1
#define SMB_RGB_BRIGHTNESS  40  // 0–255

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
  // Millis lần cuối có frame LiDAR hợp lệ / sau 1 vòng quét US (giám sát “tươi”)
  volatile uint32_t lidarLastUpdateMs;
  volatile uint32_t usLastUpdateMs;
};

extern RobotState g_state;

/** Tích lũy byte nhận từ Luna (Serial1=trước, Serial2=sau) — debug nhanh trên web (`lr1`/`lr2`). */
extern volatile uint32_t g_lunaRxBytes1;
extern volatile uint32_t g_lunaRxBytes2;

#endif // CONFIG_H
