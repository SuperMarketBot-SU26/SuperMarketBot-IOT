import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Resolve paths
    bridge_dir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
    urdf_file = os.path.join(bridge_dir, 'urdf', 'supermarketbot.urdf.xacro')
    slam_config = os.path.join(bridge_dir, 'config', 'mapper_params_online_async.yaml')
    nav2_config = os.path.join(bridge_dir, 'config', 'nav2_params.yaml')
    rviz_config = os.path.join(bridge_dir, 'config', 'rviz_config.rviz')

    # Launch Configurations
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    use_nav2 = LaunchConfiguration('use_nav2', default='false')
    agent_port = LaunchConfiguration('agent_port', default='8888')

    # 1. Micro-ROS Agent (UDP port 8888)
    microros_agent = ExecuteProcess(
        cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent', 'udp4', '--port', agent_port],
        output='screen'
    )

    # 2. Robot State Publisher (Loads URDF for TF base_link -> laser_frame @ 40cm, imu_link, wheels)
    robot_description = Command(['xacro ', urdf_file])
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': False}]
    )

    # 3. SLAM Toolbox (Online Async SLAM for YDLIDAR X3)
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_config, {'use_sim_time': False}]
    )

    # 4. RViz2 (Visualizer)
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(use_rviz),
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true', description='Launch RViz2'),
        DeclareLaunchArgument('use_nav2', default_value='false', description='Launch Nav2 stack'),
        DeclareLaunchArgument('agent_port', default_value='8888', description='Micro-ROS Agent UDP port'),

        microros_agent,
        robot_state_publisher,
        slam_toolbox,
        rviz2_node,
    ])
