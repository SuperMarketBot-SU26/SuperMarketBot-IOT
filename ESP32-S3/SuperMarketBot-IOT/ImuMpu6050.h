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
#define MPU6050_GYRO_CONFIG  0x1B
#define MPU6050_GYRO_ZOUT_H  0x47

static float    s_gyroBiasZ = 0.f;
static uint32_t s_lastImuTimeMs = 0;
bool            g_imuEnabled = false;

// Đọc 2 byte từ một thanh ghi I2C
static inline int16_t mpu6050Read16(uint8_t reg) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return 0; // Tránh treo nhiệm vụ điều khiển nếu đường bus I2C bị nhiễu do động cơ
  }
  Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)2);
  if (Wire.available() >= 2) {
    uint8_t h = Wire.read();
    uint8_t l = Wire.read();
    return (int16_t)((h << 8) | l);
  }
  return 0;
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

  // Cấu hình Gyro full scale range +/- 250 deg/s (Độ nhạy: 131 LSB / (deg/s))
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU6050_GYRO_CONFIG);
  Wire.write(0); 
  Wire.endTransmission();
  delay(10);

  // Hiệu chuẩn Gyro Z-axis: Đọc 250 mẫu khi robot tĩnh để tìm sai số tĩnh (bias)
  Serial.println(F("[IMU] Đang hiệu chuẩn Gyro (giữ robot đứng yên)..."));
  long sumZ = 0;
  const int numSamples = 250;
  for (int i = 0; i < numSamples; i++) {
    sumZ += mpu6050Read16(MPU6050_GYRO_ZOUT_H);
    delay(4);
  }
  s_gyroBiasZ = (float)sumZ / (float)numSamples;
  s_lastImuTimeMs = millis();
  g_imuEnabled = true;
  Serial.printf("[IMU] Hiệu chuẩn xong. Gyro Bias Z: %.3f\n", s_gyroBiasZ);
}

// Cập nhật góc Heading từ cảm biến Gyroscope
// Trả về true nếu có cập nhật mới thành công
inline bool imuMpu6050Update(float &headingRad) {
  if (!g_imuEnabled) return false;

  uint32_t now = millis();
  float dt = (float)(now - s_lastImuTimeMs) * 0.001f;
  s_lastImuTimeMs = now;

  if (dt <= 0.f || dt > 1.0f) {
    // Tránh lỗi khi lag vòng lặp
    return false;
  }

  int16_t rawZ = mpu6050Read16(MPU6050_GYRO_ZOUT_H);
  
  // Tính tốc độ góc Z (deg/s): Chia độ nhạy 131 LSB/(deg/s)
  float gyroZ = (float)(rawZ - s_gyroBiasZ) / 131.0f;
  
  // Chuyển sang rad/s
  float gyroZRad = gyroZ * (float)M_PI / 180.0f;

#if IMU_YAW_INVERTED
  gyroZRad = -gyroZRad; // Đảo chiều nếu IMU bị lắp ngược trục Z
#endif

  // Ngưỡng lọc nhiễu tĩnh (Deadband) để tránh trôi góc khi robot đứng im
  if (fabsf(gyroZRad) < 0.0015f) {
    gyroZRad = 0.f;
  }

  // Tích lũy góc xoay (Yaw)
  // Khi robot xoay trái (CCW), gyroZ có giá trị dương, góc tăng lên
  headingRad += gyroZRad * dt;

  // Giữ góc nằm trong khoảng [0, 2π)
  while (headingRad < 0.f) headingRad += 2.f * (float)M_PI;
  while (headingRad >= 2.f * (float)M_PI) headingRad -= 2.f * (float)M_PI;

  return true;
}

#endif // IMU_MPU6050_H
