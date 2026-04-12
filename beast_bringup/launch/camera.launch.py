#!/usr/bin/env python3

#This is a test file to see if the robot can communicate with the Mobile ROS app using rosbridge

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    camera_device_arg = DeclareLaunchArgument(
        'camera_device',
        default_value='/dev/video0',
        description='Camera device path'
    )

    camera_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='camera',
        parameters=[{
            'video_device': LaunchConfiguration('camera_device'),
            'image_width': 640,
            'image_height': 480,
            'framerate': 30.0,
            'pixel_format': 'yuyv',
            'camera_frame_id': 'camera_link',
            'io_method': 'mmap',
        }],
        remappings=[
            ('image_raw', '/camera/color/image_raw'),
        ],
    )

    return LaunchDescription([
        camera_device_arg,
        camera_node,
    ])