# Hướng Dẫn Chạy ROS2 Dual-Engine Trên Ubuntu (Dành Cho Máy Linux)

Tài liệu này dành cho bạn chạy máy Ubuntu (Humble 22.04 hoặc Lyrical 26.04) để kết nối với ESP32-S3 và điều khiển robot qua ROS2 / RViz2.

---

## 🚀 Bước 1: Pull Code Mới Về

Mở terminal trên Ubuntu:
```bash
cd ~/SuperMarketBot-IOT
git pull origin feature/lyrical-ros2-integration
```

---

## ⚡ Bước 2: Khởi Động Tất Cả Trong 1 Lệnh (All-In-One Launch)

Chạy lệnh sau trên Ubuntu:
```bash
ros2 launch ros2_bridge/launch/supermarketbot_bringup.launch.py
```

Lệnh này sẽ tự động khởi chạy:
1. **Micro-ROS Agent** (UDP Port 8888 kết nối ESP32-S3)
2. **Robot State Publisher** (Tải mô hình 3D robot: 28cm x 15cm, LiDAR cao 40cm)
3. **SLAM Toolbox** (Xây dựng bản đồ siêu nét từ YDLIDAR X3)
4. **RViz2** (Giao diện đồ họa 3D hiển thị Map + Robot + LiDAR + Navigation)

---

## 🎮 Bước 3: Điều Khiển & Quét Map Trên RViz2

1. **Xem Bản Đồ Real-time**: Khi ESP32 chạy, trên màn hình RViz2 sẽ xuất hiện các tia màu đỏ (`/scan`) và khung bản đồ xám-trắng (`/map`).
2. **Lái Tay Bằng Bàn Phím (Teleop)**: Mở terminal mới:
   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```
   Dùng các phím `i` (tiến), `j` (xoay trái), `l` (xoay phải), `k` (dừng) để lái robot. Bạn sẽ thấy motor ESP32 phản hồi cực kỳ mượt mà!

3. **Điều Khiển Điểm Đến (Nav2 Goal)**:
   - Nếu bật Nav2, chọn công cụ **"2D Nav Goal"** ở thanh công cụ phía trên RViz2.
   - Click và kéo chuột chỉ hướng trên bản đồ → Nav2 sẽ tự lập lộ trình (vẽ đường màu xanh) và truyền lệnh `/cmd_vel` xuống ESP32 để robot tự chạy đến đúng mục tiêu!

---

## 🛠️ Kiểm Tra Trạng Thái Lệch Kết Nối (Troubleshooting)

- Kiểm tra IP của laptop Ubuntu:
  ```bash
  ip addr show
  ```
- Nếu IP máy Ubuntu thay đổi (khác `192.168.154.120`), chỉ cần sửa `#define MICRO_ROS_AGENT_IP` trong `Config.h` trên ESP32 cho khớp mạng.
