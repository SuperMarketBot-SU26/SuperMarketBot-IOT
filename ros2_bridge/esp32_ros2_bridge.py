import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, Quaternion
from nav_msgs.msg import Odometry
import websocket
import json
import threading
import math
import time


# === YDLIDAR X3 NOISE FILTER (Tầng 1) ===================================
# YDLIDAR X3 đặc tả:
#   range_min = 0.12m, range_max = 8m
#   tần số 6-10 Hz, độ phân giải góc ~0.5°
# Tuy nhiên ESP32-S3 stream UDP/WS có thể rớt gói hoặc bị nhiễu bởi:
#   - Phản xạ kính / ánh sáng → dist > 6m
#   - Góc quét yếu ở rìa tầm quét → dist = 0 hoặc NaN
#   - Vật quá gần (≤ 0.15m) → nhiễu trigger trùng
# Nếu không lọc, slam_toolbox sẽ coi các điểm nhiễu này là FREE SPACE
# và vẽ tia raycast đâm xuyên qua tường (Ảnh 1).
LIDAR_RANGE_MIN_M = 0.15   # bỏ điểm < 15cm (sát LiDAR, nhiễu)
LIDAR_RANGE_MAX_M = 6.0    # bỏ điểm > 6m (giả định cửa hàng supermarket trong nhà)
LIDAR_MIN_VALID_PERCENT = 0.40  # nếu < 40% điểm hợp lệ → drop cả scan (sai góc cứng)


def filter_lidar_points(pts_m):
    """
    Lọc nhiễu LiDAR theo khoảng cách hợp lệ.
    Trả về list các range (đơn vị mét) — điểm nhiễu → NaN (slam_toolbox sẽ bỏ qua).
    Không cắt điểm trong mảng để giữ nguyên góc (angle_min/max, angle_increment).
    """
    out = []
    valid = 0
    for r in pts_m:
        if r is None:
            out.append(float('nan')); continue
        try:
            v = float(r)
        except (TypeError, ValueError):
            out.append(float('nan')); continue
        if not math.isfinite(v) or v <= 0:
            out.append(float('nan')); continue
        if v < LIDAR_RANGE_MIN_M or v > LIDAR_RANGE_MAX_M:
            out.append(float('nan')); continue
        out.append(v)
        valid += 1
    return out, valid


def should_drop_scan(n_total, n_valid):
    """Drop scan nếu quá ít điểm hợp lệ (sai cứng LiDAR, dây tuột, ESP32 treo)."""
    if n_total <= 0:
        return True
    return (n_valid / n_total) < LIDAR_MIN_VALID_PERCENT

class ESP32Ros2Bridge(Node):
    def __init__(self):
        super().__init__('esp32_ros2_bridge')
        
        # Declare parameters
        self.declare_parameter('esp32_ip', '192.168.0.105')
        self.declare_parameter('port_main', 81)
        self.declare_parameter('port_lidar', 82)
        self.declare_parameter('frame_id', 'laser_frame')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')

        self.ip = self.get_parameter('esp32_ip').get_parameter_value().string_value
        self.port_main = self.get_parameter('port_main').get_parameter_value().integer_value
        self.port_lidar = self.get_parameter('port_lidar').get_parameter_value().integer_value
        
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.odom_frame_id = self.get_parameter('odom_frame_id').get_parameter_value().string_value
        self.base_frame_id = self.get_parameter('base_frame_id').get_parameter_value().string_value

        # Build WebSocket URLs
        self.ws_main_url = f"ws://{self.ip}:{self.port_main}/"
        self.ws_lidar_url = f"ws://{self.ip}:{self.port_lidar}/"

        # ROS 2 Publishers & Subscribers
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        # IMU từ ESP32 (heading fused qua EKF trên firmware) → robot_localization fusion
        self.imu_pub  = self.create_publisher(Imu, '/imu/data', 10)
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.amcl_pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.slam_pose_callback, 10)

        # State cho IMU integration (tính angular_velocity từ delta heading)
        self._last_imu_heading = None
        self._last_imu_stamp_ns = None
        # Rate-limit cho /amcl_pose → WS (10Hz max)
        self._last_slam_pose_ns = None

        self.ws_main = None
        self.ws_lidar = None
        self.running = True

        self.get_logger().info(f"🚀 ESP32-S3 ROS 2 Bridge starting...")
        self.get_logger().info(f"   Main WS (Port {self.port_main}): {self.ws_main_url}")
        self.get_logger().info(f"   Lidar WS (Port {self.port_lidar}): {self.ws_lidar_url}")

        # Start WebSocket Threads
        threading.Thread(target=self._run_main_ws, daemon=True).start()
        threading.Thread(target=self._run_lidar_ws, daemon=True).start()

    def _run_main_ws(self):
        while self.running:
            try:
                self.ws_main = websocket.WebSocketApp(
                    self.ws_main_url,
                    on_open=lambda ws: self.get_logger().info("✅ Connected to ESP32 Main WS (Port 81)!"),
                    on_message=self._on_main_message,
                    on_error=lambda ws, err: self.get_logger().error(f"Main WS Error: {err}"),
                    on_close=lambda ws, code, msg: self.get_logger().warn("Main WS Disconnected. Retrying...")
                )
                self.ws_main.run_forever()
            except Exception as e:
                self.get_logger().error(f"Main WS Exception: {e}")
            time.sleep(2)

    def _run_lidar_ws(self):
        while self.running:
            try:
                self.ws_lidar = websocket.WebSocketApp(
                    self.ws_lidar_url,
                    on_open=lambda ws: self.get_logger().info("✅ Connected to ESP32 Lidar Stream WS (Port 82)!"),
                    on_message=self._on_lidar_message,
                    on_error=lambda ws, err: self.get_logger().debug(f"Lidar WS Note: {err}"),
                    on_close=lambda ws, code, msg: None
                )
                self.ws_lidar.run_forever()
            except Exception:
                pass
            time.sleep(2)

    def _on_main_message(self, ws, message):
        try:
            data = json.loads(message)
            msg_type = data.get("t")

            # 1. Telemetry / Pose data from Port 81
            if msg_type == "telemetry" or "x" in data or "pose" in data:
                self.publish_odometry(data)
                # Publish IMU từ HeadingRad (EKF-fused trên ESP32) cho robot_localization
                if "h" in data or "heading" in data or "HeadingRad" in data:
                    self.publish_imu_from_heading(data)
            
            # Fallback scan if sent on main ws
            elif msg_type == "lidar_scan" or "ranges" in data:
                self.publish_laser_scan_ranges(data.get("ranges", []))

        except Exception as e:
            pass

    def _on_lidar_message(self, ws, message):
        try:
            data = json.loads(message)
            
            # Format Port 82: {"pts": [[angle_deg, dist_mm], ...], "ox": ..., "oy": ..., "oh": ...}
            pts = data.get("pts", [])
            if pts:
                self.publish_laser_scan_pts(pts)
            
            # Odom in lidar stream
            if "ox" in data:
                self.publish_odometry_compact(data)

            # IMU heading from lidar stream payload
            if "oh" in data:
                self.publish_imu_from_heading({"h": data["oh"]})

        except Exception as e:
            pass

    def publish_laser_scan_pts(self, pts):
        if not pts:
            return

        # Bước 1: chuyển sang mét + phát hiện nhiễu
        raw_m = []
        for item in pts:
            # item là [angle_deg, dist_mm] hoặc dist_mm
            dist_mm = item[1] if isinstance(item, list) and len(item) > 1 else item
            if not dist_mm or float(dist_mm) <= 0:
                raw_m.append(None)
            else:
                raw_m.append(float(dist_mm) / 1000.0)

        # Bước 2: filter nhiễu YDLIDAR X3 (Tầng 1)
        ranges, n_valid = filter_lidar_points(raw_m)
        if should_drop_scan(len(raw_m), n_valid):
            self.get_logger().warn(
                f"LiDAR scan dropped: {n_valid}/{len(raw_m)} điểm hợp lệ (< {int(LIDAR_MIN_VALID_PERCENT*100)}%)"
            )
            return

        scan_msg = LaserScan()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = self.frame_id
        # YDLIDAR X3: 360° quét, bắt đầu từ phía sau (-π)
        scan_msg.angle_min = -math.pi
        scan_msg.angle_max = math.pi
        scan_msg.angle_increment = (2.0 * math.pi) / max(len(ranges), 1)
        # Match filter — slam_toolbox sẽ bỏ qua range ngoài [range_min, range_max]
        scan_msg.range_min = LIDAR_RANGE_MIN_M
        scan_msg.range_max = LIDAR_RANGE_MAX_M
        scan_msg.time_increment = 0.0
        scan_msg.scan_time = 0.1  # ~10 Hz

        scan_msg.ranges = ranges
        self.scan_pub.publish(scan_msg)

    def publish_laser_scan_ranges(self, ranges_raw):
        if not ranges_raw:
            return

        # Chuẩn hóa sang mét, loại dist <= 0 trước khi filter
        raw_m = [(float(r) / 1000.0 if r and float(r) > 0 else None) for r in ranges_raw]
        ranges, n_valid = filter_lidar_points(raw_m)
        if should_drop_scan(len(raw_m), n_valid):
            self.get_logger().warn(
                f"LiDAR scan dropped: {n_valid}/{len(raw_m)} điểm hợp lệ (< {int(LIDAR_MIN_VALID_PERCENT*100)}%)"
            )
            return

        scan_msg = LaserScan()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = self.frame_id
        scan_msg.angle_min = -math.pi
        scan_msg.angle_max = math.pi
        scan_msg.angle_increment = (2.0 * math.pi) / max(len(ranges), 1)
        scan_msg.range_min = LIDAR_RANGE_MIN_M
        scan_msg.range_max = LIDAR_RANGE_MAX_M
        scan_msg.time_increment = 0.0
        scan_msg.scan_time = 0.1

        scan_msg.ranges = ranges
        self.scan_pub.publish(scan_msg)

    def publish_odometry(self, data):
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = self.odom_frame_id
        odom_msg.child_frame_id = self.base_frame_id

        x = float(data.get("x", 0.0))
        y = float(data.get("y", 0.0))
        heading = float(data.get("h", data.get("heading", 0.0)))

        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.position.z = 0.0

        odom_msg.pose.pose.orientation.z = math.sin(heading / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(heading / 2.0)

        # ★ Covariance cho robot_localization:
        #   Project này KHÔNG dùng encoder → translation variance cao (0.25 m² ≈ 50cm 1-sigma).
        #   Heading EKF-fused trên ESP32 (gyro + wheel + SLAM) → variance vừa (0.05 rad² ≈ 13°).
        #   robot_localization sẽ kết hợp với /imu (chỉ yaw) và /amcl_pose để ra /odometry/filtered.
        odom_msg.pose.covariance = [
            0.25, 0.0,  0.0,  0.0,  0.0,  0.0,    # x
            0.0,  0.25, 0.0,  0.0,  0.0,  0.0,    # y
            0.0,  0.0,  1e6,  0.0,  0.0,  0.0,    # z (unknown, không dùng)
            0.0,  0.0,  0.0,  1e6,  0.0,  0.0,    # roll (unknown)
            0.0,  0.0,  0.0,  0.0,  1e6,  0.0,    # pitch (unknown)
            0.0,  0.0,  0.0,  0.0,  0.0,  0.05     # yaw (EKF-fused)
        ]

        self.odom_pub.publish(odom_msg)

    def publish_odometry_compact(self, data):
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = self.odom_frame_id
        odom_msg.child_frame_id = self.base_frame_id

        x = float(data.get("ox", 0.0))
        y = float(data.get("oy", 0.0))
        heading = float(data.get("oh", 0.0))

        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.position.z = 0.0

        odom_msg.pose.pose.orientation.z = math.sin(heading / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(heading / 2.0)

        odom_msg.pose.covariance = [
            0.25, 0.0,  0.0,  0.0,  0.0,  0.0,
            0.0,  0.25, 0.0,  0.0,  0.0,  0.0,
            0.0,  0.0,  1e6,  0.0,  0.0,  0.0,
            0.0,  0.0,  0.0,  1e6,  0.0,  0.0,
            0.0,  0.0,  0.0,  0.0,  1e6,  0.0,
            0.0,  0.0,  0.0,  0.0,  0.0,  0.05
        ]

        self.odom_pub.publish(odom_msg)

    def publish_imu_from_heading(self, data):
        """
        Tạo sensor_msgs/Imu từ heading EKF-fused từ ESP32, dùng cho robot_localization.

        Input data có thể chứa:
          - "h" hoặc "heading" hoặc "HeadingRad"  : heading (rad, [0, 2π))
        Angular velocity (gyroZ) được tính từ delta heading / delta time.
        Linear acceleration = 0 (ESP32 chưa publish accel).
        Orientation covariance: cao ở pitch/roll (-1 = unknown), trung bình ở yaw.

        Quaternion convention: chỉ xoay quanh trục Z (yaw).
        """
        heading_raw = data.get("h",
                      data.get("heading",
                      data.get("HeadingRad", None)))
        if heading_raw is None:
            return

        heading = float(heading_raw)
        # Chuẩn hóa về [-π, π] để tính delta chính xác
        heading_wrapped = math.atan2(math.sin(heading), math.cos(heading))

        now_ns = self.get_clock().now().nanoseconds

        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = "imu_link"

        # Orientation: quaternion từ yaw
        imu.orientation = Quaternion()
        imu.orientation.x = 0.0
        imu.orientation.y = 0.0
        imu.orientation.z = math.sin(heading_wrapped / 2.0)
        imu.orientation.w = math.cos(heading_wrapped / 2.0)

        # Orientation covariance: [-1, -1, yaw_var, -1, -1, -1]
        # (-1 = ROS convention cho "unknown" — robot_localization sẽ bỏ qua)
        # Yaw variance: ~0.05 rad² (~13° 1-sigma) cho phép EKF tin odom pose hơn
        yaw_var = 0.05
        imu.orientation_covariance = [
            -1.0, 0.0, 0.0,
             0.0, -1.0, 0.0,
             0.0, 0.0, yaw_var
        ]

        # Angular velocity: gyroZ = (heading_now - heading_prev) / dt
        imu.angular_velocity.x = 0.0
        imu.angular_velocity.y = 0.0
        if self._last_imu_heading is None or self._last_imu_stamp_ns is None:
            imu.angular_velocity.z = 0.0
        else:
            dt_s = (now_ns - self._last_imu_stamp_ns) / 1e9
            if 0.005 < dt_s < 5.0:
                d_heading = math.atan2(
                    math.sin(heading_wrapped - self._last_imu_heading),
                    math.cos(heading_wrapped - self._last_imu_heading)
                )
                imu.angular_velocity.z = d_heading / dt_s
            else:
                imu.angular_velocity.z = 0.0

        # Angular velocity covariance: cao ở x/y, trung bình ở z
        gyro_z_var = 0.02  # (rad/s)^2
        imu.angular_velocity_covariance = [
            -1.0, 0.0, 0.0,
             0.0, -1.0, 0.0,
             0.0, 0.0, gyro_z_var
        ]

        # Linear acceleration: ESP32 chưa publish → để 0 + covariance cao
        imu.linear_acceleration.x = 0.0
        imu.linear_acceleration.y = 0.0
        imu.linear_acceleration.z = 0.0
        imu.linear_acceleration_covariance = [
            -1.0, 0.0, 0.0,
             0.0, -1.0, 0.0,
             0.0, 0.0, -1.0
        ]

        self.imu_pub.publish(imu)

        # Cache cho lần sau
        self._last_imu_heading = heading_wrapped
        self._last_imu_stamp_ns = now_ns

    def cmd_vel_callback(self, msg: Twist):
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        joy_y = int(max(min(linear_x * 100.0, 100.0), -100.0))
        joy_x = int(max(min(-angular_z * 100.0, 100.0), -100.0))

        payload = {
            "t": "joy",
            "x": joy_x,
            "y": joy_y,
            "strafe": 0
        }
        if self.ws_main and self.ws_main.sock and self.ws_main.sock.connected:
            self.ws_main.send(json.dumps(payload))

    def slam_pose_callback(self, msg: PoseWithCovarianceStamped):
        # Rate-limit: SLAM Toolbox publish /amcl_pose ~5-10Hz, nhưng ESP32 chỉ cần 10Hz.
        now_ns = self.get_clock().now().nanoseconds
        if self._last_slam_pose_ns and (now_ns - self._last_slam_pose_ns) < 100_000_000:  # 100ms
            return
        self._last_slam_pose_ns = now_ns

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        q_z = msg.pose.pose.orientation.z
        q_w = msg.pose.pose.orientation.w
        heading_rad = 2.0 * math.atan2(q_z, q_w)

        payload = {
            "t": "slam_pose",
            "x": round(x, 3),
            "y": round(y, 3),
            "h": round(heading_rad, 3)
        }
        if self.ws_main and self.ws_main.sock and self.ws_main.sock.connected:
            self.ws_main.send(json.dumps(payload))

def main(args=None):
    rclpy.init(args=args)
    node = ESP32Ros2Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
