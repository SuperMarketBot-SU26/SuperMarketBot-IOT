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

#include <Arduino.h>

// micro-ROS Arduino
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

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
// ============================================================
// Đặt IP Ubuntu chạy micro-ros-agent
// Tìm bằng: ip addr show  → inet 192.168.x.x
#define MICRO_ROS_AGENT_IP    "192.168.154.120"
#define MICRO_ROS_AGENT_PORT  8888

// WiFi credentials (set trong Config.h hoặc hardcode)
// #define WIFI_SSID "your_ssid"
// #define WIFI_PASS "your_pass"

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
static void cmd_vel_callback(const void *msgin) {
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
    float lin = msg->linear.x;   // m/s forward
    float ang = msg->angular.z;  // rad/s rotation

    // Convert to PWM (existing motor driver)
    extern void botDrive(int16_t steer, int16_t throttle, uint16_t basePwm);
    extern void botRotateCW(uint16_t speed);
    extern void botStop();

    if (fabs(ang) > 0.05f && fabs(lin) < 0.05f) {
        // Xoay tại chỗ
        int pwm = (int)(fabs(ang) * 150.0f);
        pwm = constrain(pwm, 30, 200);
        if (ang > 0) botRotateCW(pwm);
        else {
            // Xoay ngược chiều
            extern void botRotateCCW(uint16_t speed);
            botRotateCCW(pwm);
        }
    } else if (fabs(lin) > 0.05f) {
        // Đi thẳng (PID yaw giữ heading hiện tại)
        int16_t throttle = (int16_t)(lin * 200.0f);
        throttle = constrain(throttle, -200, 200);
        botDrive(0, throttle, 180);
    } else {
        botStop();
    }
}

// ============================================================
// FILL LaserScan message từ g_x3Scan
// ============================================================
static void fill_scan_msg() {
    g_scan_msg.header.frame_id.data = (char*)"laser_frame";
    g_scan_msg.header.stamp.sec = millis() / 1000;
    g_scan_msg.header.stamp.nanosec = (millis() % 1000) * 1000000UL;

    g_scan_msg.angle_min = 0.0f;
    g_scan_msg.angle_max = 2.0f * M_PI;
    g_scan_msg.angle_increment = (2.0f * M_PI) / 360.0f;  // 360 bins cho 1 vòng
    g_scan_msg.time_increment = 0.0f;
    g_scan_msg.scan_time = 0.1f;  // 10 Hz
    g_scan_msg.range_min = 0.12f;  // 12cm
    g_scan_msg.range_max = 8.0f;   // 8m

    // Init ranges array (zero-fill)
    if (g_scan_msg.ranges.size == 0) {
        g_scan_msg.ranges.data = (float*)malloc(360 * sizeof(float));
        g_scan_msg.ranges.size = 360;
        g_scan_msg.ranges.capacity = 360;
    }

    // Zero all
    for (size_t i = 0; i < 360; i++) g_scan_msg.ranges.data[i] = 0.0f;

    // Fill từ g_x3Scan (góc theo rad, distance theo mm)
    extern X3Scan g_x3Scan;
    for (uint16_t i = 0; i < g_x3Scan.count; i++) {
        const LidarPoint &p = g_x3Scan.points[i];
        if (p.distanceMm < 120 || p.distanceMm > 8000) continue;  // filter out-of-range
        if (p.quality < 10) continue;

        int idx = (int)(p.angleRad / (2.0f * M_PI) * 360.0f);
        if (idx < 0) idx += 360;
        if (idx >= 360) idx = 359;

        float dist_m = (float)p.distanceMm / 1000.0f;
        g_scan_msg.ranges.data[idx] = dist_m;
    }
}

// ============================================================
// FILL Odometry message từ g_pose (encoder + IMU)
// ============================================================
static void fill_odom_msg() {
    extern Pose g_pose;
    static float prev_x = 0, prev_y = 0;
    static uint32_t prev_ms = 0;

    g_odom_msg.header.frame_id.data = (char*)"odom";
    g_odom_msg.child_frame_id.data = (char*)"base_link";
    g_odom_msg.header.stamp.sec = millis() / 1000;
    g_odom_msg.header.stamp.nanosec = (millis() % 1000) * 1000000UL;

    g_odom_msg.pose.pose.position.x = g_pose.x;
    g_odom_msg.pose.pose.position.y = g_pose.y;
    g_odom_msg.pose.pose.position.z = 0.0f;

    // Quaternion từ heading (yaw)
    float h = g_pose.headingRad * 0.5f;
    g_odom_msg.pose.pose.orientation.x = 0.0f;
    g_odom_msg.pose.pose.orientation.y = 0.0f;
    g_odom_msg.pose.pose.orientation.z = sinf(h);
    g_odom_msg.pose.pose.orientation.w = cosf(h);

    g_odom_msg.twist.twist.linear.x = g_pose.linearSpeed;
    g_odom_msg.twist.twist.angular.z = g_pose.angularSpeed;

    // Covariance (rough estimates)
    for (int i = 0; i < 36; i++) g_odom_msg.pose.covariance[i] = 0.0f;
    g_odom_msg.pose.covariance[0] = 0.01f;  // x
    g_odom_msg.pose.covariance[7] = 0.01f;  // y
    g_odom_msg.pose.covariance[35] = 0.03f; // yaw
}

// ============================================================
// FILL IMU message
// ============================================================
static void fill_imu_msg() {
    g_imu_msg.header.frame_id.data = (char*)"imu_link";
    g_imu_msg.header.stamp.sec = millis() / 1000;
    g_imu_msg.header.stamp.nanosec = (millis() % 1000) * 1000000UL;

    // Empty - child can fill from ImuMpu6050.h
}

// ============================================================
// INIT: Khởi tạo micro-ROS node
// ============================================================
inline bool init() {
    // 1. Set WiFi transport (UDP)
    IPAddress agent_ip;
    agent_ip.fromString(g_agent_ip.c_str());
    rmw_uros_set_custom_transport(
        false,                            // Serial=false
        (char*)"udp",                     // UDP transport
        g_agent_port,
        (char*)g_agent_ip.c_str()
    );

    // 2. Allocator + support
    g_allocator = rcl_get_default_allocator();
    rclc_support_init(&g_support, 0, NULL, &g_allocator);

    // 3. Node
    rclc_node_init_default(&g_node, "supermarketbot_esp32", "", &g_support);

    // 4. Publishers
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

    // 7. Init messages
    g_scan_msg.header.frame_id.data = (char*)"laser_frame";
    g_scan_msg.header.frame_id.size = strlen(g_scan_msg.header.frame_id.data);

    g_odom_msg.header.frame_id.data = (char*)"odom";
    g_odom_msg.header.frame_id.size = strlen(g_odom_msg.header.frame_id.data);
    g_odom_msg.child_frame_id.data = (char*)"base_link";
    g_odom_msg.child_frame_id.size = strlen(g_odom_msg.child_frame_id.data);

    g_imu_msg.header.frame_id.data = (char*)"imu_link";
    g_imu_msg.header.frame_id.size = strlen(g_imu_msg.header.frame_id.data);

    g_initialized = true;
    Serial.printf("[micro-ROS] ✅ Node ready, publishing:/scan /odom /imu/data, subscribing /cmd_vel (agent %s:%d)\n",
                  g_agent_ip.c_str(), g_agent_port);
    return true;
}

// ============================================================
// TICK: gọi mỗi loop iteration
// Spin executor + publish data
// ============================================================
inline void tick() {
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

    // 3. Publish /odom @ 50 Hz
    if (now - g_last_odom_ms >= 20) {
        g_last_odom_ms = now;
        fill_odom_msg();
        rcl_publish(&g_odom_pub, &g_odom_msg, NULL);
    }

    // 4. Publish /imu/data @ 50 Hz
    if (now - g_last_imu_ms >= 20) {
        g_last_imu_ms = now;
        fill_imu_msg();
        rcl_publish(&g_imu_pub, &g_imu_msg, NULL);
    }

    // 5. State machine for connection
    static uint32_t last_ping_ms = 0;
    if (now - last_ping_ms >= 1000) {
        last_ping_ms = now;
        // micro-ros library auto-pings agent
    }
}

// ============================================================
// SET agent IP/port (override macro)
// ============================================================
inline void setAgent(const char* ip, int port) {
    g_agent_ip = ip;
    g_agent_port = port;
}

}  // namespace microRos

#endif  // MICROROS_H
