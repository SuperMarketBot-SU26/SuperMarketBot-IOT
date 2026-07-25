#!/bin/bash
# ============================================================
# STEP 2: Test YDLIDAR X3 với ROS2 (cắm X3 qua USB)
# ============================================================

set -e
source /opt/ros/humble/setup.bash
source ~/ydlidar_slam/install/setup.bash

# 1. Check X3 có trên /dev/ttyUSB0 không?
echo "=== Check USB device ==="
ls -la /dev/ttyUSB* 2>/dev/null || echo "❌ Không thấy /dev/ttyUSB*"
echo ""
echo "Nếu không thấy, cắm X3 vào USB và chạy:"
echo "  sudo chmod 666 /dev/ttyUSB0"
echo ""

# 2. Set permission cho YDLIDAR
if [ -e /dev/ttyUSB0 ]; then
    sudo chmod 666 /dev/ttyUSB0
    echo "✅ Đã chmod /dev/ttyUSB0"
fi

# 3. Launch YDLIDAR X3
echo ""
echo "=== Launch YDLIDAR X3 (publish /scan) ==="
echo "Mở RViz2 ở terminal khác: ros2 run rviz2 rviz2"
echo "Add /scan topic, set Fixed Frame = laser_frame"
echo ""
ros2 launch ydlidar_ros2_driver ydlidar_launch.py \
    port:=/dev/ttyUSB0 \
    baud:=115200 \
    frame_id:=laser_frame
