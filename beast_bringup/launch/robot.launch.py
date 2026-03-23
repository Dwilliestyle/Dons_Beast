#!/usr/bin/env python3
"""
Main launch file for UGV Beast
Launches all essential nodes with configuration parameters
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the path to the config file
    pkg_share = get_package_share_directory('beast_bringup')
    config_file = os.path.join(pkg_share, 'config', 'beast_params.yaml')
    
    # Get path to ldlidar launch file
    ldlidar_pkg_share = get_package_share_directory('ldlidar_stl_ros2')
    ldlidar_launch = os.path.join(ldlidar_pkg_share, 'launch', 'ld19.launch.py')
    
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'
        ),
        
        # ESP32 Bridge Node
        # Handles: motor control, sensor data from ESP32
        Node(
            package='beast_bringup',
            executable='esp32_bridge.py',
            name='esp32_bridge',
            parameters=[
                config_file,
                {'use_sim_time': use_sim_time}
            ],
            output='screen',
            emulate_tty=True,
        ),
        
        # Battery Monitor Node
        # Monitors battery voltage and publishes warnings
        Node(
            package='beast_utils',
            executable='battery_monitor.py',
            name='battery_monitor',
            parameters=[
                config_file,
                {'use_sim_time': use_sim_time}
            ],
            output='screen',
            emulate_tty=True,
        ),
        
        # OLED Display Node
        # Updates the beast's OLED display with status info
        Node(
            package='beast_utils',
            executable='oled_display.py',
            name='oled_display',
            parameters=[
                config_file,
                {'use_sim_time': use_sim_time}
            ],
            output='screen',
            emulate_tty=True,
        ),
        
        # Odometry Publisher Node
        # Publishes odometry and transforms from wheel encoders
        Node(
            package='beast_motion',
            executable='odom_publisher.py',
            name='odom_publisher',
            parameters=[
                config_file,
                {'use_sim_time': use_sim_time}
            ],
            output='screen',
            emulate_tty=True,
        ),

        # IMU Filter (Madgwick)
        # Fuses accelerometer, gyroscope and magnetometer into stable orientation
        # Publishes filtered /imu/data used by odom_publisher for heading
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter',
            parameters=[{
                'use_mag': True,
                'publish_tf': False,
                'world_frame': 'enu',
                'gain': 0.1,
            }],
            remappings=[
                ('imu/data_raw', 'imu/data_raw'),
                ('imu/mag',      'imu/mag'),
                ('imu/data',     'imu/data'),
            ],
            output='screen',
        ),
        
        # Robot State Publisher
        # Publishes robot description and static transforms from URDF
        # Note: Requires beast_description package to be built and installed
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[
                {
                    'robot_description': open(
                        os.path.join(
                            get_package_share_directory('beast_description'),
                            'urdf',
                            'ugv_beast.urdf'
                        )
                    ).read(),
                    'use_sim_time': use_sim_time
                }
            ],
            output='screen',
            emulate_tty=True,
        ),
        
        # LD19 LIDAR
        # Launches LIDAR driver with correct port configuration
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ldlidar_launch),
            launch_arguments={
                'port_name': '/dev/ttyACM0',
                'frame_id': 'laser_frame'
            }.items()
        ),
    ])