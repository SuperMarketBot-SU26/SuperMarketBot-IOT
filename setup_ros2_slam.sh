#!/bin/bash
# ============================================================
# STEP 1: Setup ROS2 Humble workspace + dependencies
# Chạy trên Ubuntu 22.04, KHÔNG chạy trên Windows
# ============================================================

set -e

echo "============================================================"
echo "  STEP 1: Setup ROS2 Humble + YDLIDAR + SLAM Toolbox"
echo "============================================================"

# 1. Source ROS2
source /opt/ros/humble/setup.bash

# 2. Tạo workspace
mkdir -p ~/ydlidar_slam/src
cd ~/ydlidar_slam/src

# 3. Clone YDLIDAR ROS2 Driver (official, supports X3)
if [ ! -d "ydlidar_ros2_driver" ]; then
    echo "[1/4] Clone ydlidar_ros2_driver..."
    git clone -b humble https://github.com/YDLIDAR/ydlidar_ros2_driver.git
else
    echo "[1/4] ydlidar_ros2_driver exists, skip"
fi

# 4. Clone ros2_bridge (rosbridge_server) - cho WebSocket
if [ ! -d "rosbridge_suite" ]; then
    echo "[2/4] Clone rosbridge_suite..."
    git clone -b humble https://github.com/RobotWebTools/rosbridge_suite.git
else
    echo "[2/4] rosbridge_suite exists, skip"
fi

# 5. Cài thêm: slam_toolbox, navigation2, teleop, lidar
echo "[3/4] Install slam_toolbox + nav2 + teleop..."
sudo apt update
sudo apt install -y \
    ros-humble-slam-toolbox \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-teleop-twist-keyboard \
    ros-humble-teleop-twist-joy \
    ros-humble-rosbridge-server \
    ros-humble-tf2-tools \
    ros-humble-tf2-geometry-msgs \
    ros-humble-cv-bridge \
    python3-colcon-common-extensions

# 6. Build
cd ~/ydlidar_slam
echo "[4/4] Build workspace (this takes 2-5 minutes)..."
source /opt/ros/humble/setup.bash
colcon build --symlink-install --parallel-workers 2

echo ""
echo "============================================================"
echo "  ✅ DONE! Khởi động bằng:"
echo "  source ~/ydlidar_slam/install/setup.bash"
echo "============================================================"
