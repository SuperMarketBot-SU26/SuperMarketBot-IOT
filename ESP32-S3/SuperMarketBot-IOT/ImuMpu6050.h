/* =====================================================================
 *  ImuMpu6050.h — Đọc và xử lý góc xoay (heading) từ IMU MPU6050 qua I2C.
 *
 *  Mục tiêu: Sử dụng gyroscope trục Z để tích lũy góc heading thực tế,
 *  giảm thiểu sai số trượt bánh xe của Odometry khi robot quay tại chỗ.
 * =====================================================================*/
#ifndef IMU_MPU6050_H
#define IMU_MPU6050_H

#include "Config.h"
#include <Wire.h>

#define MPU6050_ADDR         0x68
#define MPU6050_PWR_MGMT_1   0x6B
#define MPU6050_CONFIG       0x1A
#define MPU6050_GYRO_CONFIG  0x1B
#define MPU6050_GYRO_ZOUT_H  0x47

static float    s_gyroBiasZ = 0.f;
static uint32_t s_lastImuTimeMs = 0;
bool            g_imuEnabled = false;

// Đọc 2 byte từ một thanh ghi I2C an toàn (có kiểm tra lỗi bus)
static inline bool mpu6050Read16(uint8_t reg, int16_t &outVal) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return false; // Lỗi truyền bus I2C (ví dụ do nhiễu động cơ)
  }
  Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)2);
  if (Wire.available() >= 2) {
    uint8_t h = Wire.read();
    uint8_t l = Wire.read();
    outVal = (int16_t)((h << 8) | l);
    return true;
  }
  return false;
}

// Khởi tạo MPU6050
inline void imuMpu6050Init() {
#if !USE_IMU_MPU6050
  Serial.println(F("[IMU] MPU6050 disabled in Config.h"));
  return;
#endif

  Serial.printf("[IMU] Initializing MPU6050 on I2C SDA:%d SCL:%d...\n", (int)IMU_I2C_SDA, (int)IMU_I2C_SCL);
  Wire.begin(IMU_I2C_SDA, IMU_I2C_SCL, 400000u); // Khởi tạo I2C bus tốc độ 400kHz
  Wire.setTimeOut(10); // Giới hạn timeout 10ms tránh treo block nhiệm vụ điều khiển khi bị nghẽn bus

  // Thử kết nối thiết bị
  Wire.beginTransmission(MPU6050_ADDR);
  if (Wire.endTransmission() != 0) {
    Serial.println(F("[IMU ERROR] Không tìm thấy cảm biến MPU6050 tại địa chỉ 0x68!"));
    return;
  }

  // Đánh thức MPU6050 (mặc định ngủ khi bật nguồn)
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU6050_PWR_MGMT_1);
  Wire.write(0); // Set to 0 to wake up
  Wire.endTransmission();
  delay(10);

  // Cấu hình Bộ lọc thông thấp kỹ thuật số (DLPF = 42Hz) để lọc nhiễu rung cơ học từ motor
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU6050_CONFIG);
  Wire.write(3); // DLPF_CFG = 3 -> Gyro 42Hz, Delay 4.8ms (Lọc sạch rung nhiễu)
  Wire.endTransmission();
  delay(10);

  // Cấu hình Gyro full scale range +/- 250 deg/s (Độ nhạy cao nhất: 131 LSB / (deg/s))
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU6050_GYRO_CONFIG);
  Wire.write(0); 
  Wire.endTransmission();
  delay(10);

  // Hiệu chuẩn Gyro Z-axis: Đọc 250 mẫu hợp lệ khi robot tĩnh để tìm sai số tĩnh (bias)
  Serial.println(F("[IMU] Dang hieu chuan Gyro (giu robot dung yen)..."));
  long sumZ = 0;
  int validSamples = 0;
  int16_t val = 0;
  // Thử tối đa 1000 lần đọc để thu thập 250 mẫu thành công sạch lỗi I2C
  for (int i = 0; i < 1000 && validSamples < 250; i++) {
    if (mpu6050Read16(MPU6050_GYRO_ZOUT_H, val)) {
      sumZ += val;
      validSamples++;
    }
    delay(4);
    yield(); // Feed WDT — tránh Watchdog reset khi calibrate lâu
  }

  if (validSamples > 0) {
    s_gyroBiasZ = (float)sumZ / (float)validSamples;
    s_lastImuTimeMs = millis();
    g_imuEnabled = true;
    Serial.printf("[IMU] Hieu chuan xong. Mau hop le: %d/250. Gyro Bias Z: %.3f\n", validSamples, s_gyroBiasZ);
  } else {
    s_gyroBiasZ = 0.f;
    g_imuEnabled = false;
    Serial.println(F("[IMU ERROR] Khong the doc du lieu tu MPU6050 Z-axis!"));
  }
}

// Cập nhật góc Heading từ cảm biến Gyroscope
// Trả về true nếu có cập nhật mới thành công
inline bool imuMpu6050Update(float &headingRad) {
  if (!g_imuEnabled) return false;

  uint32_t now = millis();
  float dt = (float)(now - s_lastImuTimeMs) * 0.001f;

  if (dt <= 0.f || dt > 1.0f) {
    s_lastImuTimeMs = now; // Tránh lỗi trôi góc khi lag
    return false;
  }

  int16_t rawZ = 0;
  if (!mpu6050Read16(MPU6050_GYRO_ZOUT_H, rawZ)) {
    // Nếu lỗi I2C, KHÔNG cập nhật s_lastImuTimeMs. 
    // Vòng lặp sau dt sẽ tăng lên tự động bù đắp đúng khoảng thời gian bị mất!
    return false; 
  }
  s_lastImuTimeMs = now; // Chỉ cập nhật mốc thời gian khi đọc thành công

  // Tính tốc độ góc Z (deg/s): Chia độ nhạy 131 LSB/(deg/s)
  float gyroZ = (float)(rawZ - s_gyroBiasZ) / 131.0f;
  
  // Chuyển sang rad/s
  float gyroZRad = gyroZ * (float)M_PI / 180.0f;

#if IMU_YAW_INVERTED
  gyroZRad = -gyroZRad; // Đảo chiều nếu IMU bị lắp ngược trục Z
#endif

  // Ngưỡng lọc nhiễu tĩnh (Deadband) để tránh trôi góc khi robot đứng im
  // Tăng nhẹ lên 0.0022f (khoảng 0.12 deg/s) để chống trôi triệt để khi đứng yên tĩnh
  if (fabsf(gyroZRad) < 0.0022f) {
    gyroZRad = 0.f;
  }

  // Tích lũy góc xoay (Yaw) với hệ số bù tỉ lệ
  headingRad += gyroZRad * dt * g_state.imuYawScale;

  // Giữ góc nằm trong khoảng [0, 2π)
  while (headingRad < 0.f) headingRad += 2.f * (float)M_PI;
  while (headingRad >= 2.f * (float)M_PI) headingRad -= 2.f * (float)M_PI;

  return true;
}

#endif // IMU_MPU6050_H
