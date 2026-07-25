#!/bin/bash
# ============================================================
# STEP 1 (Ubuntu): Cài micro-ros-agent cho ROS2 Humble
# ============================================================

set -e

echo "============================================================"
echo "  STEP 1: Cài micro-ros-agent (bridge ESP32 ↔ ROS2)"
echo "============================================================"

# 1. Install dependencies
sudo apt update
sudo apt install -y \
    ros-humble-ros-base \
    ros-humble-rmw-microxrcedds \
    ros-humble-ros2bag \
    ros-humble-rosbridge-server \
    ros-humble-slam-toolbox \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-teleop-twist-keyboard \
    ros-humble-teleop-twist-joy \
    python3-colcon-common-extensions

# 2. Build micro-ros-agent từ source
echo ""
echo "=== Build micro-ros-agent từ source... ==="
mkdir -p ~/microros_ws/src
cd ~/microros_ws/src

if [ ! -d "micro-ROS-Agent" ]; then
    git clone -b humble https://github.com/micro-ROS/micro-ROS-Agent.git
fi

cd ~/microros_ws
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -y --rosdistro humble
colcon build --symlink-install --packages-select micro_ros_agent

echo ""
echo "============================================================"
echo "  ✅ DONE!"
echo "  Test thử: ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888"
echo "============================================================"
