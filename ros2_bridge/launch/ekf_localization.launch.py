# =====================================================================
#  ekf_localization.launch.py — Khởi chạy robot_localization EKF
# =====================================================================
# Khởi node ekf_node với config từ ekf_config.yaml.
# Input:  /odom, /imu/data, /amcl_pose (đã có sẵn từ bridge + SLAM Toolbox)
# Output: /odometry/filtered, /odometry/filtered/odometry → slam_toolbox dùng làm odom
# =====================================================================

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # Path tuyệt đối tới ekf_config.yaml (cùng folder với launch file)
    config_file = os.path.join(
        os.path.dirname(os.path.realpath(__file__)),
        '..', 'config', 'ekf_config.yaml'
    )
    # Fallback: nếu robot_localization được cài local trong workspace, có thể truyền param trực tiếp
    # Nhưng cách trên (file cùng folder) đảm bảo đường dẫn đúng trên mọi môi trường.

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': False}
        ],
        remappings=[
            # Input — khớp với bridge
            ('/odometry/filtered', '/odometry/filtered'),
        ]
    )

    return LaunchDescription([
        ekf_node
    ])