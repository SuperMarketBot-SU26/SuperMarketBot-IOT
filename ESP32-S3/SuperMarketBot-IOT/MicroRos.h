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
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

// Message types
#include <sensor_msgs/msg/laser_scan.h>
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/imu.h>

// WiFi transport (UDP)
#include <WiFi.h>

// Forward declarations — actual types defined in YdlidarX3.h / Localization.h
// (at global scope, NOT inside microRos namespace)
extern struct X3Scan g_x3Scan;
extern struct Pose2D g_pose;

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

// Subscribers
static rcl_subscription_t g_cmd_vel_sub;

// Messages
static sensor_msgs__msg__LaserScan  g_scan_msg;
static nav_msgs__msg__Odometry      g_odom_msg;
static sensor_msgs__msg__Imu        g_imu_msg;
static geometry_msgs__msg__Twist    g_cmd_vel_msg;

// State
static bool   g_initialized = false;
static String g_agent_ip = MICRO_ROS_AGENT_IP;
static int    g_agent_port = MICRO_ROS_AGENT_PORT;
static uint32_t g_last_scan_ms = 0;
static uint32_t g_last_odom_ms = 0;
static uint32_t g_last_imu_ms = 0;

// Odometry angular velocity state
static uint32_t s_odomPrevMs = 0;
static float    s_odomPrevHeading = 0.f;

// ============================================================
// CALLBACK: nhận /cmd_vel từ ROS2 → điều khiển motor
// ============================================================
// Gate: WebUI joystick còn tươi → bỏ qua /cmd_vel từ ROS2.
// Threshold 300ms phù hợp với WebUI publish joystick ở ~10Hz.
static void cmd_vel_callback(const void *msgin) {
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
    float lin = msg->linear.x;
    float ang = msg->angular.z;

    const uint32_t nowMs = millis();
    const uint32_t joyAgeMs = (g_state.joyLastMs != 0)
        ? (nowMs - g_state.joyLastMs) : 0xFFFFFFFFu;
    if (joyAgeMs < 300u) {
        return;
    }

    if (fabs(ang) > 0.05f || fabs(lin) > 0.05f) {
        constexpr float ROS2_LIN_MAX = 1.20f;
        constexpr float ROS2_ANG_MAX = 5.00f;
        
        int16_t fwdPct = (int16_t)constrain((int)(lin / ROS2_LIN_MAX * 100.0f), -100, 100);
        // CRITICAL FIX: ROS 2 angular.z > 0 means CCW (Left). 
        // botDriveSmoothNormal turn > 0 means CW (Right). 
        // We MUST invert `ang`!
        int16_t turnPct = (int16_t)constrain((int)(-ang / ROS2_ANG_MAX * 100.0f), -100, 100);
        
        botDriveSmoothNormal(turnPct, fwdPct, PWM_MAX, true);
        
        // Cập nhật localization để EKF biết bánh xe đang quay
        locSetDriveCmd(
            (int16_t)constrain(fwdPct + turnPct, -100, 100),
            (int16_t)constrain(fwdPct - turnPct, -100, 100));
    } else {
        ::botStop();
    }
}

// ============================================================
// FILL LaserScan message từ g_x3Scan
// ============================================================
static void fill_scan_msg() {
    g_scan_msg.header.frame_id.data = (char*)"laser_frame";
    g_scan_msg.header.frame_id.size = 11;
    g_scan_msg.header.frame_id.capacity = 12;
    const uint32_t nowMs = millis();
    g_scan_msg.header.stamp.sec = nowMs / 1000;
    g_scan_msg.header.stamp.nanosec = (nowMs % 1000) * 1000000UL;

    g_scan_msg.angle_min = 0.0f;
    g_scan_msg.angle_max = 2.0f * M_PI;
    g_scan_msg.angle_increment = (2.0f * M_PI) / 360.0f;
    g_scan_msg.time_increment = 0.0f;
    g_scan_msg.scan_time = 0.1f;
    g_scan_msg.range_min = 0.12f;
    g_scan_msg.range_max = 8.0f;

    // Init ranges array — capacity fields REQUIRED for CDR deserialization
    if (g_scan_msg.ranges.data == NULL) {
        g_scan_msg.ranges.data = (float*)malloc(360 * sizeof(float));
        g_scan_msg.ranges.size = 360;
        g_scan_msg.ranges.capacity = 360;
        if (g_scan_msg.ranges.data == NULL) {
            Serial.println("[scan] malloc FAILED");
            return;
        }
    }

    for (size_t i = 0; i < 360; i++) g_scan_msg.ranges.data[i] = 0.0f;

    extern X3Scan g_x3Scan;
    for (uint16_t i = 0; i < g_x3Scan.count; i++) {
        const LidarPoint &p = g_x3Scan.points[i];
        if (p.distanceMm < 120 || p.distanceMm > 8000) continue;
        if (p.quality < 10) continue;

        int idx = (int)(p.angleRad / (2.0f * M_PI) * 360.0f);
        if (idx < 0) idx += 360;
        if (idx >= 360) idx = 359;

        float dist_m = (float)p.distanceMm / 1000.0f;
        float &slot = g_scan_msg.ranges.data[idx];
        if (slot == 0.0f || dist_m < slot) slot = dist_m;
    }
}

// ============================================================
// FILL Odometry message từ g_pose (encoder + IMU)
// ============================================================
static void fill_odom_msg() {
    extern Pose2D g_pose;
    const Pose2D &pose = g_pose;

    g_odom_msg.header.frame_id.data = (char*)"odom";
    g_odom_msg.header.frame_id.size = 4;
    g_odom_msg.header.frame_id.capacity = 5;
    g_odom_msg.child_frame_id.data = (char*)"base_link";
    g_odom_msg.child_frame_id.size = 10;
    g_odom_msg.child_frame_id.capacity = 11;
    const uint32_t nowMs = millis();
    g_odom_msg.header.stamp.sec = nowMs / 1000;
    g_odom_msg.header.stamp.nanosec = (nowMs % 1000) * 1000000UL;

    g_odom_msg.pose.pose.position.x = pose.x;
    g_odom_msg.pose.pose.position.y = pose.y;
    g_odom_msg.pose.pose.position.z = 0.0f;

    float h = pose.headingRad * 0.5f;
    g_odom_msg.pose.pose.orientation.x = 0.0f;
    g_odom_msg.pose.pose.orientation.y = 0.0f;
    g_odom_msg.pose.pose.orientation.z = sinf(h);
    g_odom_msg.pose.pose.orientation.w = cosf(h);

    const float wheelRps = ((g_state.rpmFL + g_state.rpmFR) * 0.5f) / 60.0f;
    const float linearMps = wheelRps * (float)WHEEL_CIRC_M;
    g_odom_msg.twist.twist.linear.x = linearMps;
    g_odom_msg.twist.twist.linear.y = 0.0f;
    g_odom_msg.twist.twist.linear.z = 0.0f;

    // Angular velocity from delta heading / dt
    if (s_odomPrevMs != 0) {
        const float dt = (float)(nowMs - s_odomPrevMs) / 1000.0f;
        if (dt > 0.001f && dt < 5.0f) {
            float dH = pose.headingRad - s_odomPrevHeading;
            while (dH >  (float)M_PI) dH -= 2.f * (float)M_PI;
            while (dH < -(float)M_PI) dH += 2.f * (float)M_PI;
            g_odom_msg.twist.twist.angular.x = 0.0f;
            g_odom_msg.twist.twist.angular.y = 0.0f;
            g_odom_msg.twist.twist.angular.z = dH / dt;
        }
    } else {
        g_odom_msg.twist.twist.angular.x = 0.0f;
        g_odom_msg.twist.twist.angular.y = 0.0f;
        g_odom_msg.twist.twist.angular.z = 0.0f;
    }
    s_odomPrevMs = nowMs;
    s_odomPrevHeading = pose.headingRad;

    for (int i = 0; i < 36; i++) {
        g_odom_msg.pose.covariance[i] = 0.0f;
        g_odom_msg.twist.covariance[i] = 0.0f;
    }
    g_odom_msg.pose.covariance[0] = 0.05f;
    g_odom_msg.pose.covariance[7] = 0.05f;
    g_odom_msg.pose.covariance[35] = 0.10f;
    g_odom_msg.twist.covariance[0] = 0.02f;
    g_odom_msg.twist.covariance[35] = 0.10f;
}

// ============================================================
// FILL Imu message
// ============================================================
static void fill_imu_msg() {
    extern Pose2D g_pose;
    const Pose2D &pose = g_pose;

    float h = pose.headingRad * 0.5f;
    g_imu_msg.orientation.x = 0.0f;
    g_imu_msg.orientation.y = 0.0f;
    g_imu_msg.orientation.z = sinf(h);
    g_imu_msg.orientation.w = cosf(h);

    float gyroZ = 0.f;
    bool gyroOk = false;
#if USE_IMU_MPU6050
    gyroOk = ::imuMpu6050GetGyroZ(gyroZ);
#endif
    g_imu_msg.angular_velocity.x = 0.0f;
    g_imu_msg.angular_velocity.y = 0.0f;
    g_imu_msg.angular_velocity.z = gyroOk ? gyroZ : 0.0f;
    g_imu_msg.linear_acceleration.x = 0.0f;
    g_imu_msg.linear_acceleration.y = 0.0f;
    g_imu_msg.linear_acceleration.z = 9.81f;

    for (int i = 0; i < 9; i++) {
        g_imu_msg.orientation_covariance[i] = 0.0f;
        g_imu_msg.angular_velocity_covariance[i] = 0.0f;
        g_imu_msg.linear_acceleration_covariance[i] = 0.0f;
    }
    g_imu_msg.orientation_covariance[0] = -1.0f;
    g_imu_msg.orientation_covariance[4] = -1.0f;
    g_imu_msg.orientation_covariance[8] = 0.02f;
    g_imu_msg.angular_velocity_covariance[0] = -1.0f;
    g_imu_msg.angular_velocity_covariance[4] = -1.0f;
    g_imu_msg.angular_velocity_covariance[8] = gyroOk ? 0.001f : -1.0f;
    g_imu_msg.linear_acceleration_covariance[0] = -1.0f;
    g_imu_msg.linear_acceleration_covariance[4] = -1.0f;
    g_imu_msg.linear_acceleration_covariance[8] = 0.01f;

    g_imu_msg.header.frame_id.data = (char*)"imu_link";
    g_imu_msg.header.frame_id.size = 9;
    g_imu_msg.header.frame_id.capacity = 10;
    const uint32_t nowMs = millis();
    g_imu_msg.header.stamp.sec = nowMs / 1000;
    g_imu_msg.header.stamp.nanosec = (nowMs % 1000) * 1000000UL;
}

// ============================================================
// INIT: Khởi tạo micro-ROS node
// ============================================================
inline bool init() {
#if defined(MICRO_ROS_USE_SERIAL) && (MICRO_ROS_USE_SERIAL == 1)
    set_microros_transports();
#else
    set_microros_wifi_transports(
        (char*)WiFi.SSID().c_str(),
        (char*)WiFi.psk().c_str(),
        (char*)g_agent_ip.c_str(),
        (uint16_t)g_agent_port
    );
#endif

    g_allocator = rcl_get_default_allocator();
    rclc_support_init(&g_support, 0, NULL, &g_allocator);

    rclc_node_init_default(&g_node, "supermarketbot_esp32", "", &g_support);

    rclc_publisher_init_default(
        &g_scan_pub, &g_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
        "/scan"
    );
    rclc_publisher_init_best_effort(
        &g_odom_pub, &g_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "/odom"
    );
    rclc_publisher_init_best_effort(
        &g_imu_pub, &g_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "/imu/data"
    );

    rclc_subscription_init_default(
        &g_cmd_vel_sub, &g_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/cmd_vel"
    );

    rclc_executor_init(&g_executor, &g_support.context, 1, &g_allocator);
    rclc_executor_add_subscription(
        &g_executor, &g_cmd_vel_sub, &g_cmd_vel_msg, cmd_vel_callback, ON_NEW_DATA
    );

    // Init messages — capacity fields REQUIRED for CDR deserialization
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
    Serial.printf("[micro-ROS] Node ready. agent=%s:%d\n",
                  g_agent_ip.c_str(), g_agent_port);
    return true;
}

// ============================================================
// SPIN
// ============================================================
inline void spin() {
    if (!g_initialized) return;

    rclc_executor_spin_some(&g_executor, RCL_MS_TO_NS(5));

    uint32_t now = millis();

    if (now - g_last_scan_ms >= 100) {
        g_last_scan_ms = now;
        fill_scan_msg();
        rcl_publish(&g_scan_pub, &g_scan_msg, NULL);
    }

    if (now - g_last_odom_ms >= 33) {
        g_last_odom_ms = now;
        fill_odom_msg();
        rcl_publish(&g_odom_pub, &g_odom_msg, NULL);
    }

    if (now - g_last_imu_ms >= 33) {
        g_last_imu_ms = now;
        fill_imu_msg();
        rcl_publish(&g_imu_pub, &g_imu_msg, NULL);
    }
}

inline void tick() { spin(); }

}  // namespace microRos
#else
namespace microRos {
  inline bool init() { return false; }
  inline void spin() {}
  inline void tick() {}
}
#endif

#endif  // MICROROS_H
