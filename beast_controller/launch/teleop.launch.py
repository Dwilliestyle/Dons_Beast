#!/usr/bin/env python3
"""
Teleop launch file for UGV beast
Launches joystick OR keyboard teleop for manual control
Can be launched separately or included with robot.launch.py
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the path to the config file
    pkg_share = get_package_share_directory('beast_bringup')
    config_file = os.path.join(pkg_share, 'config', 'beast_params.yaml')
    
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    joy_dev = LaunchConfiguration('joy_dev', default='/dev/input/js0')
    use_joystick = LaunchConfiguration('use_joystick', default='true')
    
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'
        ),
        
        DeclareLaunchArgument(
            'use_joystick',
            default_value='true',
            description='Use joystick teleop if true, keyboard teleop if false'
        ),
        
        # Joy Node - only launch if use_joystick is true
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[
                {
                    'device_id': 0,
                    'deadzone': 0.05,
                    'autorepeat_rate': 20.0,
                    'use_sim_time': use_sim_time
                }
            ],
            output='screen',
            emulate_tty=True,
            condition=IfCondition(use_joystick)
        ),
        
        # Joy Teleop Node - only launch if use_joystick is true
        Node(
            package='beast_controller',
            executable='joy_teleop.py',
            name='joy_teleop',
            parameters=[
                config_file,
                {'use_sim_time': use_sim_time}
            ],
            output='screen',
            emulate_tty=True,
            condition=IfCondition(use_joystick)
        ),
        
        # Keyboard Teleop Node - only launch if use_joystick is false
        Node(
            package='beast_controller',
            executable='keyboard_ctrl.py',
            name='keyboard_ctrl',
            parameters=[
                config_file,
                {'use_sim_time': use_sim_time}
            ],
            output='screen',
            emulate_tty=True,
            condition=UnlessCondition(use_joystick)
        ),
    ])