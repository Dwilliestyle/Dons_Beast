from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='beast_bringup',
            executable='esp32_bridge',
            name='esp32_bridge',
            output='screen'
        ),
        Node(
            package='beast_controller',
            executable='keyboard_ctrl',
            name='keyboard_ctrl',
            output='screen'
        ),
        Node(
            package='beast_utils',
            executable='battery_monitor',
            name='battery_monitor',
            output='screen'
        ),
    ])