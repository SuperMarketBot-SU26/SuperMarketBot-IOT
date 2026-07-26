#!/bin/bash
# ============================================================
# Setup ROS2 Lyrical workspace + dependencies
# Chạy trên Ubuntu 26.04 ("Resolute Raccoon")
# ============================================================
#
# This is the Lyrical/Ubuntu-26.04 counterpart of setup_ros2_slam.sh
# (which targets Humble/Ubuntu-22.04). Use THIS script on the same box
# as the ros2_ws workspace.
#
# Important: ROS2 Lyrical is a near-rolling distro. Several packages
# that ship in apt for Humble do NOT have apt builds for Lyrical yet
# (slam_toolbox, micro-ros-agent, navigation2). For those, the ros2_ws
# workspace vendors them from source via ros2_ws.repos — see that file
# and run `vcs import src < ros2_ws.repos` after cloning.
#
# What THIS script does (apt-available pieces only):
#   - Configures locale
#   - Adds the ROS 2 apt repo and installs ROS 2 Lyrical desktop
#   - Installs nav2, teleop, rviz2, tf2 tools, colcon
#   - Does NOT install slam_toolbox or micro_ros_agent (vendor via .repos)
#
# What you do next (vendor the missing pieces):
#   cd ~/ros2_ws
#   vcs import src < src/ros2_ws.repos      # clone slam_toolbox + micro_ros_agent
#   sudo apt update && rosdep install --from-paths src --ignore-src -y --rosdistro=lyrical
#   colcon build --symlink-install --parallel-workers 2

set -e

echo "============================================================"
echo "  STEP 1: Setup ROS2 Lyrical + nav2 + teleop (Ubuntu 26.04)"
echo "============================================================"

# 1. Source ROS2 if already installed (lets us skip reinstall on rerun)
if [[ -f /opt/ros/lyrical/setup.bash ]]; then
    # shellcheck source=/opt/ros/lyrical/setup.bash
    source /opt/ros/lyrical/setup.bash
    echo "[1/6] ROS2 Lyrical already installed — sourcing existing setup."
else
    echo "[1/6] Installing ROS2 Lyrical..."

    # Locale
    sudo apt update && sudo apt install -y locales software-properties-common curl
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8

    # ROS 2 apt repo (Ubuntu 26.04 'resolute' codename maps to a rolling-style
    # release line; the apt repo is the same as ROS 2 mainline).
    sudo add-apt-repository universe -y
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
        | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

    sudo apt update
    sudo apt install -y ros-lyrical-desktop python3-colcon-common-extensions
fi

# 2. Tools that are likely on apt for Lyrical — fail loudly if not, so the
#    operator knows to vendor them.
echo "[2/6] Installing nav2 + teleop + tf2 tools..."
sudo apt install -y \
    ros-lyrical-nav2-bringup \
    ros-lyrical-teleop-twist-keyboard \
    ros-lyrical-teleop-twist-joy \
    ros-lyrical-tf2-tools \
    ros-lyrical-tf2-geometry-msgs \
    ros-lyrical-rosbridge-server \
    ros-lyrical-cv-bridge \
    python3-colcon-common-extensions \
    python3-vcstool \
    python3-rosdep

# 3. rosdep init/update (one-time)
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    echo "[3/6] Initializing rosdep..."
    sudo rosdep init
fi
echo "[3/6] Updating rosdep..."
rosdep update

# 4. Auto-source ROS2 in new shells (optional but recommended)
if ! grep -q "source /opt/ros/lyrical/setup.bash" ~/.bashrc; then
    echo "[4/6] Adding ROS2 source line to ~/.bashrc..."
    echo "source /opt/ros/lyrical/setup.bash" >> ~/.bashrc
fi

echo
echo "============================================================"
echo "  ✅ ROS2 Lyrical installed."
echo "============================================================"
echo
echo "NEXT STEPS — vendor slam_toolbox + micro_ros_agent from source,"
echo "then build ros2_ws:"
echo
echo "  # Clone ros2_ws (workspace) — use your own path/branch as needed"
echo "  cd ~ && git clone <ros2_ws_url> ros2_ws"
echo "  cd ros2_ws"
echo
echo "  # Pull slam_toolbox + micro_ros_agent into src/"
echo "  vcs import src < src/ros2_ws.repos"
echo
echo "  # Install rosdep keys for the vendored packages"
echo "  rosdep install --from-paths src --ignore-src -y --rosdistro=lyrical"
echo
echo "  # Build"
echo "  colcon build --symlink-install --parallel-workers 2"
echo
echo "  # Verify"
echo "  ./test_sim.sh --check"
echo "  ./test_robot.sh --check"
echo
echo "============================================================"
echo "  Old Humble instructions: see setup_ros2_slam.sh (kept for reference)."
echo "============================================================"