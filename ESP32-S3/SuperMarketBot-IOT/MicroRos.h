// ===========================================================
// MicroRos.h — ESP32-S3 = micro-ROS node
// Publish: /scan (LiDAR), /odom (wheel encoder), /imu/data (MPU6050)
// Subscribe: /cmd_vel (motor speed command)
//
// Yêu cầu Arduino IDE:
//   - Board: ESP32S3 Dev Module
//   - Cài thư viện qua Library Manager:
//       • micro_ros_arduino (https://github.com/micro-ROS/micro_ros_arduino)
//   - Hoặc PlatformIO: lib_deps = micro_ros_arduino
//
// Compatible với ROS2 Lyrical (Ubuntu 26.04) và Humble (Ubuntu 22.04).
// ===========================================================

#ifndef MICROROS_H
#define MICROROS_H

#include "Config.h"
#include <Arduino.h>

#if defined(USE_MICRO_ROS) && (USE_MICRO_ROS == 1)
// Bật micro-ROS: Hãy cài thư viện micro_ros_arduino vào Arduino IDE và bỏ comment 5 dòng dưới:
 #include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>  // v2.1: rmw_uros_sync_session + epoch_millis

// Message types
#include <sensor_msgs/msg/laser_scan.h>
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/imu.h>
#include <tf2_msgs/msg/tf_message.h>

// WiFi transport (UDP)
#include <WiFi.h>

// ============================================================
// CONFIG
// Values sourced from Config.h (MICRO_ROS_AGENT_IP, MICRO_ROS_AGENT_PORT).
// Tìm bằng: ip addr show  → inet 192.168.x.x trên máy Linux chạy micro-ros-agent
// ============================================================

namespace microRos {

// micro-ROS objects
static rcl_node_t       g_node;
static rcl_allocator_t  g_allocator;
static rclc_support_t   g_support;
static rclc_executor_t  g_executor;

// Publishers
static rcl_publisher_t  g_scan_pub;
static rcl_publisher_t  g_odom_pub;
static rcl_publisher_t  g_imu_pub;
static rcl_publisher_t  g_tf_pub;

// Subscribers
static rcl_subscription_t g_cmd_vel_sub;

// Messages
static sensor_msgs__msg__LaserScan  g_scan_msg;
static nav_msgs__msg__Odometry      g_odom_msg;
static sensor_msgs__msg__Imu        g_imu_msg;
static geometry_msgs__msg__Twist    g_cmd_vel_msg;
static tf2_msgs__msg__TFMessage     g_tf_msg;

// State
static bool   g_initialized = false;
static String g_agent_ip = MICRO_ROS_AGENT_IP;
static int    g_agent_port = MICRO_ROS_AGENT_PORT;
static uint32_t g_last_scan_ms = 0;
static uint32_t g_last_odom_ms = 0;
static uint32_t g_last_imu_ms = 0;

// ============================================================
// CALLBACK: nhận /cmd_vel từ ROS2 → điều khiển motor
// ============================================================
// Sửa: /cmd_vel từ ROS2 chỉ có hiệu lực khi WebUI KHÔNG đang gửi joystick.
// Lý do: WebUI gửi `{t:"joy"}` thẳng vào g_state.cmdX/Y/Strafe (Core 0 WS),
// controlTask (Core 1) đọc cmdX/Y đó mỗi 20ms để gọi botDrive. Đường
// micro-ROS /cmd_vel (supervisor publish 1Hz) gọi botStop/botDrive ở context
// khác, đè lên lệnh WebUI.
// Hai đường không thể cùng chạy ổn định vì tần suất 20ms vs 5-10Hz:
// - Nếu gate theo MODE_MANUAL: Nav2 vẫn không lái được vì ESP32 firmware
//   g_state.mode không tự đổi khi ROS2 nav bắt đầu.
// - Nếu gate theo "joy còn tươi" (≤300ms): tay lái giữ → gate đóng →
//   WebUI thắng; tay nhả → gate mở → ROS2 Nav2 thắng tự động.
// Threshold 300ms phù hợp với WebUI publish joystick ở ~10Hz (chu kỳ 100ms)
// — dưới 3 lần drop liên tiếp thì coi như vẫn đang giữ.
static void cmd_vel_callback(const void *msgin) {
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
    float lin = msg->linear.x;   // m/s forward
    float ang = msg->angular.z;  // rad/s rotation

    // Hoisted constants — shared between rotation branch and forward branch so
    // both blocks see the same ROS2_PWM_MIN/ROS2_PWM_MAX/ang/lin scaling.
    constexpr float ROS2_ANG_MIN_RADPS = 0.05f;
    constexpr float ROS2_ANG_MAX_RADPS = 1.00f;
    constexpr float ROS2_LIN_MIN_MPS   = 0.05f;
    constexpr float ROS2_LIN_MAX_MPS   = 0.26f;  // match Nav2 max_vel_x
    constexpr int32_t ROS2_PWM_MIN     = 400;
    constexpr int32_t ROS2_PWM_MAX     = (int32_t)PWM_MAX;

#if defined(USE_MICRO_ROS) && (USE_MICRO_ROS == 1)
    // Gate: WebUI joystick còn tươi → bỏ qua /cmd_vel từ ROS2.
    // WebUI 'joy' update g_state.joyLastMs mỗi lần nhận (xem CtrlJson.h).
    const uint32_t nowMs = millis();
    const uint32_t joyAgeMs = (g_state.joyLastMs != 0)
        ? (nowMs - g_state.joyLastMs)
        : 0xFFFFFFFFu;
    if (joyAgeMs < 300u) {
        return;  // WebUI đang giữ joystick, để nó điều khiển
    }
#endif

    if (fabs(ang) > 0.05f && fabs(lin) < 0.05f) {
        // ── Xoay tại chỗ ─────────────────────────────────────────────
        // Map angular velocity (rad/s) → PWM trực tiếp, KHÔNG qua joystick
        // curve. Trước đây `pwm = constrain(ang * 150, 30, 200)` → PWM tối
        // đa 200, sau MIN_MOTOR_PWM mapping chỉ ~337/1023 — quá yếu để xoay
        // robot trên mặt đất (bánh quay trên không nhưng stall khi có tải).
        //
        // Mapping: 0.05 rad/s → ROS2_PWM_MIN, 1.0 rad/s → PWM_MAX.
        // ROS2_PWM_MIN = 400 đủ torque khởi động xoay trên mặt đất.
        const float angMag = fabsf(ang);
        int32_t pwm = (int32_t)(((angMag - ROS2_ANG_MIN_RADPS) /
                                 (ROS2_ANG_MAX_RADPS - ROS2_ANG_MIN_RADPS)) *
                                (ROS2_PWM_MAX - ROS2_PWM_MIN) + ROS2_PWM_MIN);
        if (pwm < ROS2_PWM_MIN) pwm = ROS2_PWM_MIN;
        if (pwm > ROS2_PWM_MAX) pwm = ROS2_PWM_MAX;
        // Debug log — verify mapping mỗi 200ms trong lúc xoay.
        static uint32_t s_lastRotLog = 0;
        if (millis() - s_lastRotLog > 200u) {
          s_lastRotLog = millis();
          int32_t outPwm = 130 + (pwm * (1023 - 130)) / 1023;
          Serial.printf("[cmd_vel] ROTATE ang=%.3f → in_pwm=%ld → out_pwm=%ld/1023\n",
                        ang, (long)pwm, (long)outPwm);
        }
        if (ang > 0) ::botRotateCW((uint16_t)pwm);
        else         ::botRotateCCW((uint16_t)pwm);
    } else if (fabs(lin) > 0.05f) {
        // ── Đi thẳng / rẽ trong khi tiến ──────────────────────────────
        // Map linear velocity (m/s) → PWM thẳng, KHÔNG qua joystick curve.
        // Trước đây `botDrive(0, throttle, 180)` với throttle=lin*200 đi qua
        // quadratic curve: lin=0.20 (Nav2 desired_linear_vel) → throttle=40 →
        // yCurve=16 → PWM=29. Sau MIN_MOTOR_PWM mapping chỉ ~194/1023, không
        // đủ torque để vượt ma sát tĩnh.
        //
        // Mapping: 0.05 m/s → ROS2_PWM_MIN, 0.26 m/s (Nav2 max_vel_x) → PWM_MAX.
        // Tuyến tính, đơn giản, dễ kiểm.
        const float linMag = fabsf(lin);
        int32_t fwdPwm = (int32_t)(((linMag - ROS2_LIN_MIN_MPS) /
                                    (ROS2_LIN_MAX_MPS - ROS2_LIN_MIN_MPS)) *
                                   (ROS2_PWM_MAX - ROS2_PWM_MIN) + ROS2_PWM_MIN);
        if (fwdPwm < ROS2_PWM_MIN) fwdPwm = ROS2_PWM_MIN;
        if (fwdPwm > ROS2_PWM_MAX) fwdPwm = ROS2_PWM_MAX;

        // Tính PWM riêng từng bên cho kết hợp lin + ang (differential drive).
        // Tỉ lệ ang/lin → chênh PWM 2 bên. Với lin nhỏ nhất (0.05 m/s) và ang
        // max (1 rad/s), chênh có thể vượt fwdPwm → cap về 0 cho bên lùi.
        // Nav2 hầu như không gửi lin rất nhỏ + ang lớn cùng lúc (controller
        // giảm lin khi xoay), nên đây là trường hợp hiếm.
        int32_t rotPwm = (int32_t)(fabsf(ang) *
                                   (ROS2_PWM_MAX / ROS2_ANG_MAX_RADPS) * 0.5f);
        if (rotPwm > fwdPwm) rotPwm = fwdPwm;  // không để bên nào âm khi xoay trong khi tiến chậm
        int32_t leftPwm, rightPwm;
        if (ang >= 0) {
            // Rẽ phải: bên trái nhanh hơn
            leftPwm  = fwdPwm + rotPwm;
            rightPwm = fwdPwm - rotPwm;
        } else {
            // Rẽ trái: bên phải nhanh hơn
            leftPwm  = fwdPwm - rotPwm;
            rightPwm = fwdPwm + rotPwm;
        }
        if (leftPwm  < 0) leftPwm  = 0;
        if (rightPwm < 0) rightPwm = 0;
        if (leftPwm  > ROS2_PWM_MAX) leftPwm  = ROS2_PWM_MAX;
        if (rightPwm > ROS2_PWM_MAX) rightPwm = ROS2_PWM_MAX;

        // Đảo chiều nếu lin âm (lùi)
        if (lin < 0) {
            leftPwm  = -leftPwm;
            rightPwm = -rightPwm;
        }

        // Debug log — verify mapping mỗi 200ms trong lúc tiến.
        static uint32_t s_lastFwdLog = 0;
        if (millis() - s_lastFwdLog > 200u) {
          s_lastFwdLog = millis();
          int32_t outL = (leftPwm > 0)  ? (130 + (leftPwm  * 893) / 1023) : 0;
          int32_t outR = (rightPwm > 0) ? (130 + (rightPwm * 893) / 1023) : 0;
          Serial.printf("[cmd_vel] FWD lin=%.3f ang=%.3f → L_in=%ld R_in=%ld → L_out=%ld R_out=%ld/1023\n",
                        lin, ang, (long)leftPwm, (long)rightPwm, (long)outL, (long)outR);
        }

        // Báo Localization để EKF wheel update dùng cho heading fusion.
        // Trả về -100..+100 % so với PWM_MAX.
        locSetDriveCmd(
            (int16_t)constrain((int)(leftPwm  * 100L / ROS2_PWM_MAX), -100, 100),
            (int16_t)constrain((int)(rightPwm * 100L / ROS2_PWM_MAX), -100, 100));

        const int32_t sp[4] = {leftPwm, leftPwm, rightPwm, rightPwm};
        ::motorApplyLayout(sp);
    } else {
        ::botStop();
    }
}

// ============================================================
// FILL LaserScan message từ g_x3Scan
// ============================================================
static void fill_scan_msg() {
    // Update header string fields on every publish
    g_scan_msg.header.frame_id.data = (char*)"laser_frame";
    g_scan_msg.header.frame_id.size = 11;
    g_scan_msg.header.frame_id.capacity = 12;
    // v2.2: revert to millis()-based stamps (see fill_odom_msg for rationale)
    const uint32_t nowMs_scan = millis();
    g_scan_msg.header.stamp.sec     = nowMs_scan / 1000u;
    g_scan_msg.header.stamp.nanosec = (nowMs_scan % 1000u) * 1000000UL;

    g_scan_msg.time_increment = 0.0f;
    g_scan_msg.scan_time = 0.1f;  // 10 Hz
    g_scan_msg.range_min = 0.12f;  // 12cm
    g_scan_msg.range_max = 8.0f;   // 8m

    // Init ranges array only. With MQTT+WiFi+micro-ROS already eating heap
    // (~8KB largest contiguous block at boot), even 2KB for ranges is risky.
    // `intensities` is OPTIONAL in sensor_msgs/LaserScan (slam_toolbox ignores it),
    // so leave it empty (size=0) to save 8KB heap.
    if (g_scan_msg.ranges.data == NULL) {
        const size_t total_bins = 360;  // Was 2000; heap too tight
        g_scan_msg.ranges.data = (float*)malloc(total_bins * sizeof(float));
        if (g_scan_msg.ranges.data == NULL) {
            Serial.println("[scan] malloc(1440) FAILED — heap exhausted, skip /scan");
            g_scan_msg.ranges.size = 0;
            g_scan_msg.ranges.capacity = 0;
            return;
        }
        g_scan_msg.ranges.size = total_bins;
        g_scan_msg.ranges.capacity = total_bins;
        // intensities: intentionally size=0 — ROS serializer handles empty sequences
    }

    // Skip publish entirely if allocation failed
    if (g_scan_msg.ranges.size == 0 || g_scan_msg.ranges.data == NULL) {
        return;
    }

    // Zero all bins
    for (size_t i = 0; i < g_scan_msg.ranges.size; i++) {
        g_scan_msg.ranges.data[i] = 0.0f;
    }

    // Update angle params to match new bin count
    g_scan_msg.angle_min = 0.0f;
    g_scan_msg.angle_max = 2.0f * M_PI;
    g_scan_msg.angle_increment = (2.0f * M_PI) / (float)g_scan_msg.ranges.size;

    // Fill từ ::g_x3Scan (góc theo rad, distance theo mm).
    // Định nghĩa ở global namespace (YdlidarX3.h). Dùng `::` để tránh bị
    // nhầm với microRos::g_x3Scan (nếu có). YdlidarX3.h đã được include
    // trước MicroRos.h trong .ino nên ::g_x3Scan đã có ở scope này.
    const X3Scan &scan = ::g_x3Scan;
    for (uint16_t i = 0; i < scan.count; i++) {
        const LidarPoint &p = scan.points[i];
        if (p.distanceMm < 120 || p.distanceMm > 8000) continue;  // filter out-of-range
        if (p.quality < 10) continue;

        int idx = (int)(p.angleRad / (2.0f * M_PI) * (float)g_scan_msg.ranges.size);
        if (idx < 0) idx += g_scan_msg.ranges.size;
        if (idx >= (int)g_scan_msg.ranges.size) idx = g_scan_msg.ranges.size - 1;

        float dist_m = (float)p.distanceMm / 1000.0f;
        // Use min() — closest point wins this bin (slam_toolbox prefers shorter range)
        float &slot = g_scan_msg.ranges.data[idx];
        if (slot == 0.0f || dist_m < slot) slot = dist_m;
    }
}

// ============================================================
// FILL Odometry message từ g_pose (Localization) + encoder + IMU.
// v2.1 (2026-07-27): FIX zero angular_velocity bug.
//
// Bug trước đây: twist.twist.angular.z = 0.0f (hardcode) → EKF
// robot_localization fusion sai → RViz không xoay theo robot.
// Fix: angular.z = (Δheading EKF) / dt trong khoảng giữa 2 lần publish.
// ============================================================
static uint32_t s_odomPrevMs = 0;
static float    s_odomPrevHeading = 0.f;

static void fill_odom_msg() {
    const Pose2D &pose = ::g_pose;

    // ---- Header ----
    g_odom_msg.header.frame_id.data = (char*)"odom";
    g_odom_msg.header.frame_id.size = 4;
    g_odom_msg.header.frame_id.capacity = 5;
    g_odom_msg.child_frame_id.data = (char*)"base_link";
    g_odom_msg.child_frame_id.size = 10;
    g_odom_msg.child_frame_id.capacity = 11;
    // v2.2: revert to millis()-based stamps. EKF + slam consume this OK
    // because use_sim_time=false and they fuse on receipt. /tf publication
    // (year-1970-stamped) is no longer published from ESP32.
    const uint32_t nowMs_od = millis();
    g_odom_msg.header.stamp.sec     = nowMs_od / 1000u;
    g_odom_msg.header.stamp.nanosec = (nowMs_od % 1000u) * 1000000UL;

    // ---- Pose ----
    g_odom_msg.pose.pose.position.x = pose.x;
    g_odom_msg.pose.pose.position.y = pose.y;
    g_odom_msg.pose.pose.position.z = 0.0f;
    const float h = pose.headingRad * 0.5f;
    g_odom_msg.pose.pose.orientation.x = 0.0f;
    g_odom_msg.pose.pose.orientation.y = 0.0f;
    g_odom_msg.pose.pose.orientation.z = sinf(h);
    g_odom_msg.pose.pose.orientation.w  = cosf(h);

    // ---- Twist linear: từ RPM trung bình (Localization / Odometry) ----
    const float wheelRps = ((g_state.rpmFL + g_state.rpmFR) * 0.5f) / 60.0f;
    const float linearMps = wheelRps * (float)WHEEL_CIRC_M;
    g_odom_msg.twist.twist.linear.x = linearMps;
    g_odom_msg.twist.twist.linear.y = 0.0f;
    g_odom_msg.twist.twist.linear.z = 0.0f;

    // ---- Twist angular: từ delta heading / dt (FIX bug) ----
    // Heading đã được fusion bởi imuFusion::step → g_pose.headingRad (EKF 1D).
    // Tính angular.z = (h_now - h_prev) / dt rad/s.
    if (s_odomPrevMs != 0) {
        const float dt = (float)(nowMs_od - s_odomPrevMs) / 1000.0f;
        if (dt > 0.001f && dt < 5.0f) {
            float dH = pose.headingRad - s_odomPrevHeading;
            // wrap về [-π, π] để tránh 2π jump
            while (dH >  (float)M_PI) dH -= 2.f * (float)M_PI;
            while (dH < -(float)M_PI) dH += 2.f * (float)M_PI;
            g_odom_msg.twist.twist.angular.x = 0.0f;
            g_odom_msg.twist.twist.angular.y = 0.0f;
            g_odom_msg.twist.twist.angular.z = dH / dt;
        }
    } else {
        // Publish đầu tiên — chưa có dt, gửi 0.
        g_odom_msg.twist.twist.angular.x = 0.0f;
        g_odom_msg.twist.twist.angular.y = 0.0f;
        g_odom_msg.twist.twist.angular.z = 0.0f;
    }
    s_odomPrevMs      = nowMs_od;
    s_odomPrevHeading = pose.headingRad;

    // ---- Covariance: tune theo hardware. Pose.gps-style diag. ----
    for (int i = 0; i < 36; i++) {
        g_odom_msg.pose.covariance[i]    = 0.0f;
        g_odom_msg.twist.covariance[i]   = 0.0f;
    }
    g_odom_msg.pose.covariance[0]  = 0.05f;  // x (5cm 1-sigma)
    g_odom_msg.pose.covariance[7]  = 0.05f;  // y
    g_odom_msg.pose.covariance[35] = 0.10f;  // yaw (≈5.7°)
    g_odom_msg.twist.covariance[0] = 0.02f;  // vx
    g_odom_msg.twist.covariance[35] = 0.10f; // vyaw
}

// ============================================================
// FILL Imu message từ MPU6050 gyro Z thật + fused heading quaternion.
// v2.1 (2026-07-27): FIX all-zero bug.
//
// Bug trước đây: angular_velocity = linear_acceleration = (0,0,0)
// hardcode → ROS2 EKF không hợp nhất được gyro → SLAM heading.
// Fix:
//   - angular_velocity.z = gyro Z thô đã trừ bias (rad/s), từ imuMpu6050GetGyroZ().
//   - linear_acceleration.z = 9.81 m/s² (gravity, robot đứng yên trên sàn).
//   - Covariance -1 cho unknown components (X,Y theo sensor_msgs spec).
// ============================================================
static uint32_t s_imuPrevGyroZ_ms __attribute__((unused)) = 0;
static float    s_imuPrevGyroZ_radps __attribute__((unused)) = 0.f;

static void fill_imu_msg() {
    const Pose2D &pose = ::g_pose;

    // ---- Orientation: heading quaternion từ EKF fusion ----
    const float h = pose.headingRad * 0.5f;
    g_imu_msg.orientation.x = 0.0f;
    g_imu_msg.orientation.y = 0.0f;
    g_imu_msg.orientation.z = sinf(h);
    g_imu_msg.orientation.w  = cosf(h);

    // ---- angular_velocity.z: gyro thô từ MPU6050 (rad/s) ----
    // x,y = 0 (chỉ dùng 1 trục Z cho heading fusion).
    float gyroZ = 0.f;
    bool gyroOk = false;
#if USE_IMU_MPU6050
    gyroOk = ::imuMpu6050GetGyroZ(gyroZ);
#endif
    g_imu_msg.angular_velocity.x = 0.0f;
    g_imu_msg.angular_velocity.y = 0.0f;
    g_imu_msg.angular_velocity.z = gyroOk ? gyroZ : 0.0f;

    // ---- linear_acceleration: chỉ set Z = gravity khi đứng yên trên sàn ----
    // MPU6050 accelerometer chưa được expose qua API, mặc định coi như đứng yên.
    // (-Z axis convention: sensor reports -9.81 m/s² on Z when upright; nhưng
    // ROS convention is +9.81 on Z for ENU → +X forward, +Y left, +Z up.)
    g_imu_msg.linear_acceleration.x = 0.0f;
    g_imu_msg.linear_acceleration.y = 0.0f;
    g_imu_msg.linear_acceleration.z = 9.81f;

    // ---- Covariance theo sensor_msgs/Imu spec ----
    // Diag entries = variance (1-sigma²). Off-diag = 0.
    // Linear_acceleration.x và y đánh -1 (unknown) vì MPU6050 API chưa expose.
    for (int i = 0; i < 9; i++) {
        g_imu_msg.orientation_covariance[i]          = 0.0f;
        g_imu_msg.angular_velocity_covariance[i]     = 0.0f;
        g_imu_msg.linear_acceleration_covariance[i]  = 0.0f;
    }
    g_imu_msg.orientation_covariance[0] = -1.0f;  // roll  unknown
    g_imu_msg.orientation_covariance[4] = -1.0f;  // pitch unknown
    g_imu_msg.orientation_covariance[8] = 0.02f;  // yaw   var ≈ (0.14 rad/s)² ~ 0.02
    g_imu_msg.angular_velocity_covariance[0] = -1.0f;  // wx unknown
    g_imu_msg.angular_velocity_covariance[4] = -1.0f;  // wy unknown
    g_imu_msg.angular_velocity_covariance[8] = gyroOk ? 0.001f : -1.0f;  // wz
    g_imu_msg.linear_acceleration_covariance[0] = -1.0f;  // ax unknown
    g_imu_msg.linear_acceleration_covariance[4] = -1.0f;  // ay unknown
    g_imu_msg.linear_acceleration_covariance[8] = 0.01f;  // az (assume gravity is correct)

    g_imu_msg.header.frame_id.data = (char*)"imu_link";
    g_imu_msg.header.frame_id.size = 9;
    g_imu_msg.header.frame_id.capacity = 10;
    // v2.2: revert to millis()-based stamps (see fill_odom_msg for rationale)
    const uint32_t nowMs_imu = millis();
    g_imu_msg.header.stamp.sec     = nowMs_imu / 1000u;
    g_imu_msg.header.stamp.nanosec = (nowMs_imu % 1000u) * 1000000UL;
}

// ============================================================
// INIT: Khởi tạo micro-ROS node
// ============================================================
inline bool init() {
    // 1. Set WiFi transport (UDP)
    set_microros_wifi_transports(
        (char*)WiFi.SSID().c_str(),
        (char*)WiFi.psk().c_str(),
        (char*)g_agent_ip.c_str(),
        (uint16_t)g_agent_port
    );

    // 2. Allocator + support
    g_allocator = rcl_get_default_allocator();
    rclc_support_init(&g_support, 0, NULL, &g_allocator);

    // 3. Node
    rclc_node_init_default(&g_node, "supermarketbot_esp32", "", &g_support);

    // (v2.2) Removed rmw_uros_sync_session call. On this micro-ros_arduino +
    // WiFi UDP build, the function returns OK but doesn't actually offset the
    // clock — rmw_uros_epoch_millis() returns MCU uptime, not synced epoch.
    // Stamps use millis() throughout; consumers tolerate this when /tf is not
    // published from ESP32.

    // 4. Publishers
    // QoS strategy: KEEP ESP32 DEFAULTS, fix mismatch on ROS2 side.
    // micro_ros_arduino's default (RELIABLE) works reliably on ESP32; the
    // BEST_EFFORT variant allocates memory and silently fails on heap-
    // constrained ESP32 (seen: zero publishers for all topics).
    // The slam_toolbox mismatch (subscriber wants BEST_EFFORT) is fixed by
    // a relay node on the ROS2 side, not by changing the ESP32.
    rclc_publisher_init_default(
        &g_scan_pub, &g_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
        "/scan"
    );
    rclc_publisher_init_default(
        &g_odom_pub, &g_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "/odom"
    );
    rclc_publisher_init_default(
        &g_imu_pub, &g_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "/imu/data"
    );
    rclc_publisher_init_default(
        &g_tf_pub, &g_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage),
        "/tf"
    );

    // 5. Subscribers
    rclc_subscription_init_default(
        &g_cmd_vel_sub, &g_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/cmd_vel"
    );

    // 6. Executor (1 sub + 0 timers, callback handle intervals)
    rclc_executor_init(&g_executor, &g_support.context, 1, &g_allocator);
    rclc_executor_add_subscription(
        &g_executor, &g_cmd_vel_sub, &g_cmd_vel_msg, cmd_vel_callback, ON_NEW_DATA
    );

    // 7. Init messages — capacity fields are REQUIRED by micro-ROS CDR
    // deserialization. Without them, agent reads garbage past the null terminator.
    g_scan_msg.header.frame_id.data = (char*)"laser_frame";
    g_scan_msg.header.frame_id.size = 11;
    g_scan_msg.header.frame_id.capacity = 12;

    g_odom_msg.header.frame_id.data = (char*)"odom";
    g_odom_msg.header.frame_id.size = 4;
    g_odom_msg.header.frame_id.capacity = 5;
    g_odom_msg.child_frame_id.data = (char*)"base_link";
    g_odom_msg.child_frame_id.size = 10;
    g_odom_msg.child_frame_id.capacity = 11;

    g_imu_msg.header.frame_id.data = (char*)"imu_link";
    g_imu_msg.header.frame_id.size = 9;
    g_imu_msg.header.frame_id.capacity = 10;

    g_initialized = true;
    Serial.printf("[micro-ROS] Node ready. agent=%s:%d\n", g_agent_ip.c_str(), g_agent_port);

    // Diagnostic state at init — capture for [Diag] log periodic.
    Serial.printf("[micro-ROS] IMU enabled=%d, encoder enabled=%d, heap=%u\n",
#if USE_IMU_MPU6050
                  g_imuEnabled ? 1 : 0,
#else
                  0,
#endif
#if USE_ENCODER_HARDWARE
                  1,
#else
                  0,
#endif
                  (unsigned)ESP.getFreeHeap());
    return true;
}

// ============================================================
// SPIN: gọi mỗi loop iteration
// Spin executor + publish data
// v2.1 (2026-07-27): ADD /tf publish + diagnostic health log.
// v2.2 (2026-07-28): ROLL BACK /tf publish + sync_session. ESP32 has no RTC;
//     rmw_uros_epoch_millis() on this micro-ros_arduino+UDP build returns MCU
//     uptime (ms) not actually-synced epoch, so tf2 rejects all /tf frames as
//     "from the past" (32 days into the year 1970). The robust fix: STOP
//     publishing /tf from ESP32. robot_state_publisher (static URDF TFs) +
//     ekf_node (converts /odom → odom→base_link TF) handle TF on ROS2 side.
//     /odom, /imu/data, /scan still use millis()-based stamps; EKF and slam
//     have always accepted those silently when /tf wasn't part of the chain.
// ============================================================
inline void spin() {
    if (!g_initialized) return;

    // 1. Spin executor (xử lý /cmd_vel callback)
    rclc_executor_spin_some(&g_executor, RCL_MS_TO_NS(5));

    uint32_t now = millis();

    // 2. Publish /scan @ 10 Hz (mỗi 100ms)
    if (now - g_last_scan_ms >= 100) {
        g_last_scan_ms = now;
        fill_scan_msg();
        rcl_publish(&g_scan_pub, &g_scan_msg, NULL);
    }

    // 3. Publish /odom @ 30 Hz (mỗi ~33 ms).
    //    v2.1: angular.z computed from Δheading — see fill_odom_msg().
    if (now - g_last_odom_ms >= 33) {
        g_last_odom_ms = now;
        fill_odom_msg();
        rcl_publish(&g_odom_pub, &g_odom_msg, NULL);
    }

    // 4. Publish /imu/data @ 30 Hz (mỗi ~33 ms).
    //    v2.1: angular_velocity.z từ MPU6050 gyro thô — see fill_imu_msg().
    if (now - g_last_imu_ms >= 33) {
        g_last_imu_ms = now;
        fill_imu_msg();
        rcl_publish(&g_imu_pub, &g_imu_msg, NULL);
    }

    // (v2.2) /tf publication REMOVED. ekf_node owns odom→base_link TF.
    //        robot_state_publisher owns base_link→{laser,imu,wheels} static TFs.

    // 5. State machine for connection
    static uint32_t last_ping_ms = 0;
    if (now - last_ping_ms >= 1000) {
        last_ping_ms = now;
        // micro-ros library auto-pings agent
    }

    // 6. DIAGNOSTIC LOG: in trạng thái sensor mỗi 3s để bắt lỗi zero.
    static uint32_t last_diag_ms = 0;
    if (now - last_diag_ms >= 3000) {
        last_diag_ms = now;

        float gyroZ = 0.f;
        bool imuOk = false;
#if USE_IMU_MPU6050
        imuOk = ::imuMpu6050GetGyroZ(gyroZ);
#endif
        const float wheelRpmAvg = (g_state.rpmFL + g_state.rpmFR) * 0.5f;
        const float wheelMps = wheelRpmAvg * (float)WHEEL_CIRC_M / 60.0f;
        // Compute angular.z quickly for log
        float angZ = 0.f;
        if (s_odomPrevMs != 0) {
            const float dt = (float)(now - s_odomPrevMs) / 1000.0f;
            if (dt > 0.001f && dt < 5.0f) {
                float dH = g_pose.headingRad - s_odomPrevHeading;
                while (dH >  (float)M_PI) dH -= 2.f * (float)M_PI;
                while (dH < -(float)M_PI) dH += 2.f * (float)M_PI;
                angZ = dH / dt;
            }
        }
        Serial.printf("[Diag] imu=%d gyroZ=%+.3f rad/s | wheelAvg=%.1f rpm (%.3f m/s) | "
                      "yaw=%+.2f° | odom.lin.x=%+.3f odom.ang.z=%+.3f | "
                      "encL=%ld encR=%ld\n",
                      imuOk ? 1 : 0, gyroZ,
                      wheelRpmAvg, wheelMps,
                      g_pose.headingRad * 180.f / (float)M_PI,
                      g_odom_msg.twist.twist.linear.x,
                      angZ,
                      (long)::odomGetTicksL(), (long)::odomGetTicksR());
    }
}

inline void tick() { spin(); }

}  // namespace microRos
#else
namespace microRos {
  inline bool init() { return false; }
  inline void spin() {}
  inline void tick() {}
  inline void setAgent(const char* ip, int port) {}
}
#endif

#endif  // MICROROS_H
