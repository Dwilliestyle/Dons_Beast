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
            package='beast_motion',
            executable='odom_publisher',
            name='odom_publisher',
            output='screen'
        ),
        Node(
            package='beast_utils',
            executable='battery_monitor',
            name='battery_monitor',
            output='screen'
        ),
    ])