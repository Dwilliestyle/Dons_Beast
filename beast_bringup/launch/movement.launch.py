from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='beast_bringup',
            executable='esp32_bridge.py',
            name='esp32_bridge',
            output='screen'
        ),
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter',
            output='screen',
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
            ]
        ),
        Node(
            package='beast_motion',
            executable='odom_publisher',
            name='odom_publisher',
            output='screen',
            parameters=[{
                'use_imu_heading': True,
            }]
        ),
        Node(
            package='beast_utils',
            executable='battery_monitor',
            name='battery_monitor',
            output='screen'
        ),
    ])