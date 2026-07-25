# SmartMarketBot — ROS 2 WSL2 Integration Guide

HƯỚNG DẪN TÍCH HỢP HỆ THỐNG ROS 2 TRÊN WSL2 (UBUNTU 22.04 LTS)

---

## 🚀 Bước 1: Cài đặt WSL2 (Thực hiện trên Windows PowerShell Admin)

Mở PowerShell với quyền Administrator và chạy:
```powershell
wsl --install -d Ubuntu-22.04
```
> Sau khi cài đặt xong, hãy mở Ubuntu-22.04 từ Start Menu và thiết lập `username` và `password`.

---

## 📦 Bước 2: Cài đặt ROS 2 Humble & Dependencies (Thực hiện trong Ubuntu WSL2 Terminal)

Copy và paste toàn bộ các lệnh dưới đây vào Terminal của Ubuntu:

```bash
# 1. Cấu hình Locale
sudo apt update && sudo apt install locales software-properties-common curl -y
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# 2. Thêm ROS 2 Repository
sudo add-apt-repository universe -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 3. Cài đặt ROS 2 Humble & Nav2 / Cartographer / ROSBridge
sudo apt update
sudo apt install ros-humble-desktop ros-humble-rosbridge-suite ros-humble-nav2-bringup ros-humble-cartographer-ros python3-pip -y

# 4. Tự động load môi trường ROS 2
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## ⚡ Bước 3: Chạy Bridge Node kết nối Robot ESP32-S3 với ROS 2

1. Cài thư viện `websocket-client`:
   ```bash
   pip3 install websocket-client
   ```

2. Truy cập vào thư mục `ros2_bridge` từ WSL2:
   ```bash
   cd "/mnt/e/FPT UNIVERSITY/CN9/Sep401/SuperMarketBot-IOT/ros2_bridge"
   ```

3. Chạy Node Bridge (Thay `192.168.1.X` bằng địa chỉ IP WiFi thực tế của ESP32-S3):
   ```bash
   python3 esp32_ros2_bridge.py --ros-args -p esp32_ws_url:="ws://192.168.1.X/ws"
   ```

---

## 📊 Bước 4: Kiểm tra dữ liệu trên ROS 2 Topics

Trong một cửa sổ Terminal mới trên Ubuntu WSL2:

* **Xem dữ liệu LaserScan từ YDLIDAR X3:**
  ```bash
  ros2 topic echo /scan
  ```
* **Xem Odometry Pose của Robot:**
  ```bash
  ros2 topic echo /odom
  ```
* **Gửi lệnh di chuyển ROS 2:**
  ```bash
  ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"
  ```
* **Mở Rviz2 xem 2D Laser Scan & Robot Pose:**
  ```bash
  rviz2
  ```
