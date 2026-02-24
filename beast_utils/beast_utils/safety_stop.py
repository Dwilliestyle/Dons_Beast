#!/usr/bin/env python3
import math
import subprocess
from enum import Enum

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class State(Enum):
    FREE = 0
    DANGER = 1


class SafetyStop(Node):
    def __init__(self):
        super().__init__('safety_stop_node')
        self.declare_parameter('danger_distance', 0.3)
        self.declare_parameter('scan_topic', 'scan')
        self.declare_parameter('safety_stop_topic', 'safety_stop')
        
        self.danger_distance = self.get_parameter('danger_distance').get_parameter_value().double_value
        self.scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.safety_stop_topic = self.get_parameter('safety_stop_topic').get_parameter_value().string_value
        
        self.state = State.FREE
        self.prev_state = State.FREE

        self.laser_sub = self.create_subscription(
            LaserScan, self.scan_topic, self.laser_callback, 10
        )
        self.safety_stop_pub = self.create_publisher(
            Bool, self.safety_stop_topic, 10
        )
        
        self.get_logger().info(f'Safety Stop initialized - danger distance: {self.danger_distance}m')

    def speak(self, text):
        """Use espeak to speak the warning"""
        try:
            subprocess.Popen(['espeak', text], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception as e:
            self.get_logger().warn(f'Failed to speak: {e}')

    def laser_callback(self, msg: LaserScan):
        self.state = State.FREE

        for range_value in msg.ranges:
            if not math.isinf(range_value) and range_value <= self.danger_distance:
                self.state = State.DANGER
                break

        # Only publish when state changes
        if self.state != self.prev_state:
            is_safety_stop = Bool()
            is_safety_stop.data = (self.state == State.DANGER)
            self.safety_stop_pub.publish(is_safety_stop)
            
            if self.state == State.DANGER:
                self.get_logger().warn('DANGER! Obstacle detected - stopping robot')
                self.speak('Warning! Obstacle detected. Stopping.')
            else:
                self.get_logger().info('Clear - resuming normal operation')
                self.speak('Path clear. Resuming.')
            
            self.prev_state = self.state


def main():
    rclpy.init()
    node = SafetyStop()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()