/* =====================================================================
 *  Config.h — SmartMarketBot Mini 4WD
 *  Board: ESP32-S3-DevKitC-1 (N16R8) — theo sơ đồ Espressif
 *  Tránh: GPIO 19,20 (USB D+/D-), 33-37 (Octal PSRAM trên module N16R8)
 *  Phải tránh: GPIO 38 = RGB LED nội bộ — KHÔNG dùng cho TB6612
 * =====================================================================*/
#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

/* ---------------------- ĐỘNG LỰC (2x TB6612FNG) ---------------------
 *
 * Sơ đồ chuẩn (module TB6612FNG): mỗi IC có 2 kênh H-bridge.
 *   • VM, GND: nguồn động cơ (VD: 2S–3S ~7–12 V); GND chung với ESP.
 *   • VCC: 2,7–5,5 V logic; PWMA/PWMB nhận PWM từ ESP (3,3 V OK).
 *   • STBY = HIGH: mở driver; LOW: toàn IC ngủ — **cả 2 IC nối STBY → GPIO47**.
 *   • Kênh A: AIN1, AIN2 hướng; PWMA tốc độ; ra motor: **AO1 & AO2** (2 dây motor).
 *   • Kênh B: BIN1, BIN2; PWMB; ra motor: **BO1 & BO2**.
 *
 * TB6612 #1 — gắn với bánh bên TRÁI xe:
 *   Kênh A → motor góc **Trái trước (FL)**  |  Kênh B → **Trái sau (RL)**
 * TB6612 #2 — bên PHẢI:
 *   Kênh A → **Phải trước (FR)**  |  Kênh B → **Phải sau (RR)**
 *
 * Nếu motor quay ngược khi lệnh “tiến”: đổi 2 dây AO1↔AO2 (hoặc BO1↔BO2)
 * *hoặc* bật **Đảo chiều** trên web cho góc đó (không cần hàn lại).
 * Nếu 2 kênh của 1 IC bị đổi chỗ với nhau: dùng web **hoán vị kênh** cho 2 góc tương ứng.
 * -------------------------------------------------------------------- */
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

/* -------------------- YDLIDAR X3 (360°, 30m, SLAM) ------------------ */
// Dùng Serial1 riêng để không xung đột TF-Luna. ESP32-S3 có 3 UART:
//   Serial0 = USB (debug)
//   Serial1 = LIDAR_X3_TX=15, LIDAR_X3_RX=16 (dùng cho X3)
//   Serial2 = LIDAR_B_TX=1,  LIDAR_B_RX=2  (TF-Luna back)
// Lưu ý: ENC_RL=16 đang dùng; nếu bật X3 phải tắt encoder RL hoặc chuyển ENC_RL sang GPIO khác.
#define USE_YDLIDAR_X3          1   // 1 = Bật đọc YDLidar X3 trực tiếp trên ESP32-S3
#define YDLIDAR_X3_TX           43  // ESP TX (GPIO 43) → Nối sang RX của YDLIDAR Hub
#define YDLIDAR_X3_RX           44  // ESP RX (GPIO 44) ← Nối sang TX của YDLIDAR Hub
#define YDLIDAR_X3_BAUD         115200  // Baudrate chuẩn của YDLIDAR X3
#define YDLIDAR_SCAN_HZ         10   // Tần số scan (10 Hz điển hình cho X3)
#define YDLIDAR_SCAN_BUFF_SIZE  1024  // Buffer bytes cho mỗi scan packet
#define YDLIDAR_MAX_POINTS      720  // 360° / 0.5° resolution ≈ 720 points (X3 thực tế ~640 points)
/**
 * TF-Luna (Benewake): gửi lệnh UART sau khi mở cổng — bật output, khung 9 byte (cm), FPS, save.
 * Tắt (=0) nếu bạn đã cấu hình bằng tool PC và không muốn firmware đụng vào.
 */
#define TFLUNA_SEND_INIT_CMD  1
/** Tần số mẫu 1–250 Hz (chỉ khi TFLUNA_SEND_INIT_CMD=1). */
#define TFLUNA_SAMPLE_HZ      100

/* -------------------- SIÊU ÂM (4x HC-SR04 — 4 góc xe) ---------------- */
// VCC 5V, GND; Trig 3,3V OK; Echo 5V → chia áp 3,3V (1k+2k) vào GPIO
// Mặc định: Echo 10=Trái trước, 11=Trái sau, 12=Phải trước, 13=Phải sau + Trig 14 chung.
#define US_TRIG         14
#define US_ECHO_LF      10
#define US_ECHO_RL      11
#define US_ECHO_RF      12
#define US_ECHO_RR      13
#define US_ECHO_F       US_ECHO_LF
#define US_ECHO_B       US_ECHO_RL
#define US_ECHO_L       US_ECHO_RF
#define US_ECHO_R       US_ECHO_RR
#define US_PING_MAX_CM    200   // Max range for HC-SR04 (200cm = ~11.4ms timeout per ping, safe for 50ms loop)
/** Nghỉ giữa hai ping (ms) — TRIG chung, tránh cross-talk. */
#define US_INTER_PING_MS  16u
#define US_DISPLAY_MAX_CM 200
/** Dưới ngưỡng này (cm) coi là không đo được / nhiễu SR04. */
#define US_MIN_VALID_CM     2
/**
 * 1 = 4× HC-SR04 (né vật theo 4 góc). 0 = TF-Luna trước/sau.
 */
#define USE_HC_SR04_HARDWARE  1
/** Bật cả hai phần cứng hoạt động song song để Fusion 5 cảm biến. */
#define USE_LIDAR_HARDWARE    0

#if USE_HC_SR04_HARDWARE
/** Dừng cứng & khẩn cấp (cm) — yêu cầu: < 35 cm thì dừng. */
#define US_STOP_CM            35
/** Bắt đầu lách trước khi chạm vùng dừng. */
#define US_OA_DETECT_CM       50
/** Đủ xa để tiến / coi bên trống (SR04 không cần 1 m như LiDAR). */
#define US_PATH_CLEAR_CM      50
#define OA_DETECT_CM          US_OA_DETECT_CM
#define PATH_CLEAR_MIN_CM     US_PATH_CLEAR_CM
#define OA_PATH_CLEAR_STREAK  18
#else
#define US_STOP_CM            30
#define US_OA_DETECT_CM       42
#define US_PATH_CLEAR_CM      48
#endif

/* -------------------- CẢM BIẾN GÓC IMU MPU6050 (I2C) ---------------- */
#define USE_IMU_MPU6050  1
#define IMU_I2C_SDA      17    // Chân I2C SDA (Đã chuyển sang GPIO 17 an toàn, tránh 35/36 trùng PSRAM)
#define IMU_I2C_SCL      18    // Chân I2C SCL (Đã chuyển sang GPIO 18 an toàn, tránh 35/36 trùng PSRAM)
#define IMU_YAW_INVERTED 1     // Đặt thành 1 nếu Robot bị xoay tại chỗ vô hạn (do cảm biến IMU bị lật ngược)

/* -------------------- ENCODER (cảm biến gạt/MH, DO nối ESP) ------- */
#define USE_ENCODER_HARDWARE  0 // 1 = Bật đọc encoder bánh xe; 0 = Tắt (khi tháo rời phần cứng tránh nhiễu chân floating)
// DO → GPIO + interrupt; 3,3/5V theo lô module (thường 3,3V OK)
// GPIO3 (ENC_FR): trên S3 là MTCK/JTAG — trong Arduino chọn USB JTAG disabled / peripheral JTAG off nếu encoder không đếm xung.
#define ENC_FL        39    // Trước trái
#define ENC_RL        16    // Sau trái
#define ENC_FR        3     // Trước phải (JTAG, chỉ dùng input)
#define ENC_RR        48    // Sau phải (DevKitC-1: LED RGB = 38, không dùng 38 cho enc)

// Số xung trên 1 vòng bánh xe (tuỳ đĩa encoder - thường 20 khe chữ U)
#define ENC_PPR       20.0f
// Chu vi bánh xe (mét) để tính quãng đường — ví dụ bánh D=65mm
#define WHEEL_DIAM_M  0.065f
#define WHEEL_CIRC_M  (PI * WHEEL_DIAM_M)
// Hệ số hiệu chuẩn quãng đường (giảm < 1.0 nếu đi xa hơn lý thuyết, tăng > 1.0 nếu đi ngắn hơn)
#define ODOM_CALIB_FACTOR  1.0f

/* -------------------- PWM / LEDC ----------------------------------- */
#define PWM_FREQ      20000 // 20kHz, ngoài ngưỡng nghe
#define PWM_RES_BITS  10    // 0..1023
#define PWM_MAX       ((1 << PWM_RES_BITS) - 1)

/* -------------------- CÂN BẰNG LỰC KÉO TRÁI/PHẢI (Motor Trim) ------
 *
 * Robot 4WD có thể lệch phải/trái do bất đối xứng cơ khí giữa 2 bên bánh:
 *  - Bánh mòn không đều, motor khác tốc độ danh định, dây dẫn khác điện trở.
 *  - Scale < 1.0 = giảm công suất bên đó, > 1.0 = tăng (hiếm khi cần).
 *
 * Auto-calibrate (NV1c) sẽ tự điều chỉnh LEFT_MOTOR_SCALE trong khoảng
 * [MOTOR_SCALE_MIN..MOTOR_SCALE_MAX] dựa trên yaw drift của IMU.
 *
 * Giá trị dưới đây là **khởi đầu an toàn** (=1.0 = không bias). Sau khi robot
 * chạy thử & tune thủ công qua web (nếu có), NVS sẽ lưu lại. Reset = xoá NVS.
 * -------------------------------------------------------------------- */
#define LEFT_MOTOR_SCALE_DEFAULT   1.00f   // Scale bánh Trái (FL+RL) — tắt trim tạm, để 1.00
#define RIGHT_MOTOR_SCALE_DEFAULT  1.00f   // Scale bánh Phải (FR+RR)
/** Safety clamp — không cho scale vượt quá ±20% để tránh brick robot */
#define MOTOR_SCALE_MIN            0.80f
#define MOTOR_SCALE_MAX            1.20f
/** NVS keys (lưu vào flash, giữ qua reboot) */
#define NVS_KEY_SCALE_L            "motScL"
#define NVS_KEY_SCALE_R            "motScR"

/* -------------------- AUTO-CALIBRATE YAW DRIFT (NV1c) ---------------
 *
 * Khi đi thẳng (cruise, cmdX≈0, cmdY≠0), IMU báo yaw drift tích lũy > ngưỡng
 * → tự tinh chỉnh LEFT_MOTOR_SCALE để bù. Tính theo dạng tích phân đơn giản
 * (P-controller trên drift) — KHÔNG dùng PID đầy đủ vì drift noise cao.
 *
 * Cách hoạt động:
 *   1. Mỗi AUTO_CAL_INTERVAL_MS, lấy yaw drift trung bình (deg/sec).
 *   2. driftRate > AUTO_CAL_THRESH_DEGPS → scaleL giảm (robot lệch phải → trái mạnh quá → giảm trái).
 *      driftRate < -AUTO_CAL_THRESH_DEGPS → scaleL tăng (robot lệch trái → tăng trái).
 *   3. Mỗi lần điều chỉnh tối đa AUTO_CAL_STEP (0.5%/tick) → không giật.
 *   4. Lưu NVS mỗi AUTO_CAL_SAVE_MS (5 giây) — tránh ghi flash quá nhiều.
 *   5. Chỉ chạy khi MODE_AUTO/MODE_WAYPOINT (không lái tay).
 *
 * Debug: Telemetry JSON sẽ có `calDrift` (deg/s) + `calScaleL/R` để web hiển thị.
 * -------------------------------------------------------------------- */
#define AUTO_CAL_ENABLE               0     // 0 = tắt auto-calibrate (dùng scale thủ công)
                                          // Bật =1 khi đã hiệu chỉnh cứng xong motor trim cố định
#define AUTO_CAL_INTERVAL_MS          1000u // Đo drift mỗi 1 giây
#define AUTO_CAL_SAVE_MS              5000u // Ghi NVS mỗi 5 giây
#define AUTO_CAL_THRESH_DEGPS         2.0f  // Ngưỡng drift tối thiểu (deg/s) để kích hoạt
#define AUTO_CAL_STEP                 0.005f// Mỗi tick điều chỉnh tối đa 0.5%
#define AUTO_CAL_DEAD_ZONE_DEGPS      0.5f  // Drift < ngưỡng này thì bỏ qua (nhiễu IMU)

/* -------------------- AN TOÀN NÉ VẬT CẢN (không gian mở: siêu thị / hành lang) - */
// Vùng "dừng cứng" (cm): dùng trong tự lái + né tránh. Tăng 26–35 nếu nhiễu/dừng sớm quá.
#define SAFE_STOP_CM    15
// Bắt đầu giảm tốc khi vật trước gần hơn ngưỡng (cm). 80–120 = ít nhạy từ xa; 180–220 = nhả ga sớm.
#define SAFE_SLOW_CM    100

/** Đọc LiDAR dưới ngưỡng này (cm) coi là nhiễu / sàn / ngoài tầm tin cậy TF-Luna (~20cm min). */
#define LIDAR_MIN_VALID_CM      18
#define SAFE_LOOP_MS    50    // Chu kỳ vòng an toàn (ms) — 30→50ms để WebIO có CPU thở
/** Ngưỡng trái/phải (cm) để bẻ lái trong AUTO — chỉ có tác dụng khi bật HC-SR04 (USE_HC_SR04_HARDWARE=1). */
#define SAFE_SIDE_AVOID_CM  30

#if USE_HC_SR04_HARDWARE
#define AUTO_LIDAR_BLOCK_CM     US_STOP_CM
#else
#define AUTO_LIDAR_BLOCK_CM     16
#endif
#if !USE_HC_SR04_HARDWARE
/** Phát hiện vật cản — LiDAR (cm). */
#define OA_DETECT_CM            70
#define PATH_CLEAR_MIN_CM       100
#define OA_PATH_CLEAR_STREAK    12
#endif
#define OA_CLEAR_MIN_CM         PATH_CLEAR_MIN_CM
#define AUTO_LIDAR_CLEAR_CM     PATH_CLEAR_MIN_CM
#define ROBOT_HEAVY_LOAD        1
#if ROBOT_HEAVY_LOAD
#define AUTO_CRUISE_SPEED_PCT   50
#define AUTO_MIN_PWM_FRAC       22
#else
#define AUTO_CRUISE_SPEED_PCT   40
#define AUTO_MIN_PWM_FRAC       12
#endif
/** 0 = chỉ LiDAR trước (khuyến nghị chạy sàn — sau hay đọc sàn → dừng liên tục). 1 = cả sau. */
#define AUTO_LIDAR_BLOCK_USE_REAR 0

/** Legacy / khi USE_HC_SR04_HARDWARE=1 thêm bẻ cạnh; LiDAR-only chỉ dùng AUTO_LIDAR_* ở FSM chính */
#define AUTO_US_SLOW_CM      85   // (SR04) trước gần hơn → giảm ga tiến
#define AUTO_US_SIDE_CM      22   // cạnh (chỉ SR04 4 hướng)
#define AUTO_US_BACK_STOP_CM 26   // (backup cũ — giữ define nếu tái dùng)
#define AUTO_BACKUP_MS       400u
#define AUTO_TURN_MS         550u
/** Phase 1 — Backup khi OA blocked (lùi thử) */
#define AUTO_BACKUP_REVERSE_MS  600u   // Thời gian lùi (ms)
#define AUTO_BACKUP_SPEED_PCT   45     // Tốc độ lùi (% của PWM_MAX)

/** Phase 1 — Stuck detection (motor chạy nhưng encoder không quay) */
#define STUCK_CHECK_INTERVAL_MS  500u  // Kiểm tra mỗi 500ms
#define STUCK_THRESHOLD          3     // 3 lần liên tiếp (~1.5s) = bị kẹt
#define STUCK_MIN_PWM            200   // Chỉ check khi PWM đang đủ lớn

/* -------------------- LOCAL OBSTACLE AVOIDANCE (Phase 3.5) --------- */
/** Góc xoay tối đa mỗi chiều khi quét 2 bên (độ) */
#define OA_SCAN_ANGLE_DEG       50.0f
/** Tốc độ xoay khi scan (% PWM_MAX) — chậm để LiDAR đọc kịp */
#define OA_SCAN_SPEED_PCT       42
/** Góc lái chéo khi tránh vật cản (độ) */
#define OA_SWERVE_ANGLE_DEG     35.0f
/** Quãng đường lái chéo sang bên trống (m) — robot nặng cần lệch xa hơn */
#define OA_SWERVE_DIST_M        (ROBOT_HEAVY_LOAD ? 0.52f : 0.40f)
/** Tốc độ khi lái chéo và đi vượt (% PWM_MAX) */
#define OA_SWERVE_SPEED_PCT     (ROBOT_HEAVY_LOAD ? 45 : 32)
/** Quãng đường đi thẳng để vượt qua vật cản (m) */
#define OA_PASS_DIST_M          (ROBOT_HEAVY_LOAD ? 0.65f : 0.50f)
/** Số lần thử tự tránh tối đa trước khi fallback chờ/reroute */
#define OA_MAX_ATTEMPTS         2
/** Thời gian fallback chờ trước khi xin reroute (ms) */
#define OA_FALLBACK_WAIT_MS     10000u
/* -------------------- AUTO-DOCKING (Phase 3.5) --------------------- */
/** Node ID trạm sạc trong database (phải khớp seed data) */
#define DOCK_NODE_ID            2
/** Ngưỡng pin yếu kích hoạt auto-dock (%) */
#define DOCK_LOW_BAT_PCT        20
/** Ngưỡng pin đầy để reset dock flag (%) */
#define DOCK_FULL_BAT_PCT       80

/* -------------------- WIFI SOFTAP ---------------------------------- */
/** Tablet + ESP32-CAM (STA) cùng vào mạng này. CAM: ESP32-CAM/Config.h WIFI_* */
#define AP_SSID         "SmartMarketBot"
#define AP_PASS         "12345678"

/* -------------------- WIFI STA (kết nối router để MQTT) ----------- */
/** Robot thử lần lượt từng WiFi — kết nối được cái đầu tiên tìm thấy.
 *  Thêm hotspot điện thoại vào STA_SSID_2/3/4/5 để demo ở bất kỳ đâu mà không cần reflash. */
#define STA_SSID               "FPTH_Student"     // Ưu tiên 1 - WiFi trường FPT
#define STA_PASS               "hoithanghieu"
#define STA_SSID_2             "Snuggie"        // Hotspot điện thoại demo (tránh trùng AP của ESP)
#define STA_PASS_2             "asksnuggie"
#define STA_SSID_3             "Tran Coffee 1"    // Dự phòng / quán cafe (ưu tiên 3)
#define STA_PASS_3             "xincamon@"
#define STA_SSID_4             "0917"                 // Ưu tiên 4 — điền SSID + PASS bên dưới
#define STA_PASS_4             "vinhhung.125"
#define STA_SSID_5             "FPTU_HallA"       // Ưu tiên 5 - WiFi Lab FPT
#define STA_PASS_5             "12345678"
#define STA_CONNECT_TIMEOUT_MS 10000u   // Timeout mỗi SSID (ms) — giảm xuống để thử nhanh hơn
#define STA_MAX_RETRIES        3        // Số lần thử mỗi SSID trước khi sang SSID tiếp theo
/** Kênh 2.4 GHz (1–11). 6 thường ít chồng lấn; tránh kênh “lạ” nếu điện thoại lọc theo vùng. */
#define AP_WIFI_CHANNEL 6
#define AP_MAX_CLIENTS  4
#define WEB_PORT        80
#define WEB_SSL_PORT    443   /* HTTPS — camera tablet (getUserMedia) */
#define WS_PORT         81
/** Chu kỳ broadcast WebSocket (ms). ≥500 = mượt trên điện thoại qua AP. */
#define WEB_WS_PERIOD_MS        500u
/** 0 = tắt HTTPS khi chỉ cần lái robot (tiết kiệm RAM/CPU). 1 = bật /vision camera. */
#define VISION_HTTPS_ENABLE     0
/** Sau boot: ép MANUAL + không nhận Auto/Waypoint/MQTT navigate (ms). */
#define BOOT_GUARD_MS           12000u
/** 0 = chỉ SoftAP (web mượt). 1 = thêm STA + MQTT (HiveMQ Cloud hoặc local broker). */
#define WIFI_STA_ENABLE         1
/** 0 = Tắt hẳn kết nối MQTT (tránh treo khi chạy offline/local). 1 = Bật MQTT. */
#define MQTT_ENABLE             1

/* -------------------- ĐO PIN (ADC, tùy chọn) ------------------------
 *  ESP chỉ đọc được 0..~3.3 V trên chân ADC — cần chiết áp 2 điện trở từ nguồn
 *  muốn theo dõi (khuyến nghị: điểm 12 V trước / sau pin, GND chung với ESP).
 *  Không nên chỉ đo 5 V sau XL4015 để suy % pin: buck ổn định 5 V trong khi 12 V
 *  vẫn sụt — % trên web sẽ không đúng.
 *
 *  Ví dụ: 12V → R1(68k) → chân ADC → R2(10k) → GND
 *         Vadc = Vbat * R2/(R1+R2)  (đặt BAT_DIV_* khớp R thật của bạn)
 *  Bật BAT_MONITOR_ENABLE 1 và đặt BAT_ADC_PIN trùng chân còn trống + hỗ trợ ADC. */
#define BAT_MONITOR_ENABLE  1
/** Mặc định tắt. Bật BAT thì đặt chân ADC trống (không trùng enc/LiDAR/UART). */
#define BAT_ADC_PIN         15
#define BAT_DIV_R1_KOHM     68.0f
#define BAT_DIV_R2_KOHM     10.0f
/** Ngưỡng V pin (tại điểm đo) — chỉnh theo loại pin (3S Li-ion, 12V SLA, …). */
#define BAT_V_FULL          12.6f
#define BAT_V_EMPTY         10.2f

/* -------------------- LED: chỉ RGB zin sẵn trên bo DevKit (WS2812, GPIO 38) --- */
// Không dùng thêm bóng / dải LED ngoài — 2× LiDAR, 4× enc, 2× driver; SR04 tùy chọn (USE_HC_SR04_HARDWARE).
// Trùng GPIO: 38 = LED zin, ENC_RR=48; (SR04) Echo 10–13 Trig 14; ENC_FL = 39.
#define SMB_ONBOARD_RGB     1
#define SMB_NEOPIXEL_PIN    38
#define SMB_NEOPIXEL_COUNT  1
#define SMB_RGB_BRIGHTNESS  40  // 0–255

/* -------------------- LƯU TRỮ -------------------------------------- */
#define NVS_NAMESPACE "smb"

enum RobotMode : uint8_t {
  MODE_MANUAL   = 0,    // Lái tay
  MODE_AUTO     = 1,    // Tự hành né vật cản (reactive FSM)
  MODE_WAYPOINT = 2     // Tự hành bám waypoint (Pure Pursuit, Phase 3)
};

// (enum WheelMode đã bỏ — hệ thống chỉ dùng differential drive (bánh thường))

/* -------------------- CẤU TRÚC CHIA SẺ GIỮA 2 CORE ----------------- */
struct RobotState {
  volatile uint8_t wheelMode __attribute__((unused)); // Giữ để tránh sửa layout NVS cũ, không còn ý nghĩa
  // Cảm biến khoảng cách (cm)
  volatile int16_t usFront, usBack, usLeft, usRight;
  /** 4 góc xe (sau remap web) — Trái trước / Trái sau / Phải trước / Phải sau */
  volatile int16_t usLF, usLR, usRF, usRR;
  volatile int16_t lidarFront, lidarBack;
  // Tốc độ bánh xe (RPM)
  volatile float rpmFL, rpmRL, rpmFR, rpmRR;
  // Quãng đường (m) ước lượng
  volatile float distFL, distRL, distFR, distRR;
  // Điều khiển
  volatile int16_t cmdX;      // -100..100 (trái/phải)
  volatile int16_t cmdY;      // -100..100 (tiến/lùi)
  volatile int16_t cmdStrafe;  // -100..100 (trượt ngang, Mecanum)
  volatile uint16_t baseSpeed;    // 0..PWM_MAX — lái tay + mặc định khi chưa chỉnh auto
  volatile uint16_t autoBaseSpeed;// 0..PWM_MAX — tốc độ nền riêng cho tự hành (slider web)
  volatile uint16_t swerveBaseSpeed; // 0..PWM_MAX — tốc độ riêng khi dạt tránh/lùi/xoay (slider web)
  volatile uint16_t rotateBaseSpeed; // 0..PWM_MAX — tốc độ xoay hướng (slider web)
  volatile float imuYawScale;        // Hệ số nhân bù góc IMU (mặc định 1.0f)
  volatile RobotMode mode;
  volatile bool estop;        // Cờ dừng khẩn cấp
  // Millis lần cuối có frame LiDAR hợp lệ / sau 1 vòng quét US (giám sát “tươi”)
  volatile uint32_t lidarLastUpdateMs;
  volatile uint32_t usLastUpdateMs;
  /** Tốc độ PWM thực tế đã xuất ở chu kỳ trước (sau slew limiter) — index theo MotorId (TB6612 vật lý).
   *  Chia sẻ giữa taskControl (Core 1) và taskWebIO (Core 0) nên KHÔNG dùng static cục bộ. */
  volatile int32_t lastMotorSpeed[4];

  // Configs tunable online
  volatile float alignThresholdDeg;    // mặc định 10.0f
  volatile uint16_t rotateSpeedMinPct; // mặc định 10
  volatile uint16_t usStopCm;          // mặc định 30
  volatile uint16_t usOaDetectCm;      // mặc định 42
  volatile uint16_t usPathClearCm;     // mặc định 48
  volatile uint16_t usPathClearStreak; // mặc định 18
  volatile float yawKp;                // mặc định 40.0f
  volatile float yawKi;                // mặc định 0.0f
  volatile float yawKd;                // mặc định 2.0f
  /** Motor trim scales (NV1a). Volatile vì WebUI/MQTT có thể cập nhật runtime. */
  volatile float leftMotorScale;       // 0.80..1.20, mặc định 1.00
  volatile float rightMotorScale;      // 0.80..1.20, mặc định 1.00
};

/** FSM tự hành (AN_*) — hiển thị trên Web/MQTT khi không cắm USB Serial. */
extern volatile uint8_t g_autoFsmState;

/** LiDAR cm hợp lệ (bỏ 0 / nhiễu / dưới tầm tin cậy TF-Luna). */
inline bool lidarCmValid(int16_t cm) {
  return cm > (int16_t)LIDAR_MIN_VALID_CM;
}

inline bool lidarFrontBlocked(int16_t fCm) {
  return lidarCmValid(fCm) && fCm < (int16_t)AUTO_LIDAR_BLOCK_CM;
}

inline bool lidarRearBlocked(int16_t bCm) {
  return (AUTO_LIDAR_BLOCK_USE_REAR != 0) && lidarCmValid(bCm)
      && bCm < (int16_t)AUTO_LIDAR_BLOCK_CM;
}

extern RobotState g_state;

/** Tích lũy byte nhận từ Luna (Serial1=trước, Serial2=sau) — debug nhanh trên web (`lr1`/`lr2`). */
extern volatile uint32_t g_lunaRxBytes1;
extern volatile uint32_t g_lunaRxBytes2;

/** Dò “có tín hiệu thật” cho HMI — millis lần cuối (0 = chưa từng). Định nghĩa trong Sensors.h / Odometry.h */
extern volatile uint32_t g_luna1LastOkMs;
extern volatile uint32_t g_luna2LastOkMs;
extern volatile uint32_t g_usPhyLastEchoMs[4];
extern volatile uint32_t g_encPhyLastPulseMs[4];
extern volatile uint32_t s_settleUntilMs;

/** Cửa sổ thời gian: sau bấy lâu không có tín hiệu thì web hiển thị OFF */
#define SENSOR_LINK_MS_LIDAR  500u
#define SENSOR_LINK_MS_US     2000u
#define SENSOR_LINK_MS_ENC    3500u

extern bool g_imuEnabled;

/** NV1c — Auto-calibrate motor trim state accessor (avoid multi-def). */
struct MotorTrimState;
extern MotorTrimState& motorTrimInstance();

#include "LogStream.h"

#endif // CONFIG_H
