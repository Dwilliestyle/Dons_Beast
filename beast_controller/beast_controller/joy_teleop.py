#!/usr/bin/env python3
# encoding: utf-8

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import pygame


def get_joystick_names():
    """Detect connected joysticks and return their names"""
    pygame.init()
    pygame.joystick.init()

    joystick_names = []
    joystick_count = pygame.joystick.get_count()
    
    if joystick_count == 0:
        print("No joystick detected")
    else:
        for i in range(joystick_count):
            joystick = pygame.joystick.Joystick(i)
            joystick.init()
            joystick_names.append(joystick.get_name())

    pygame.quit()
    return joystick_names


class JoyTeleop(Node):
    def __init__(self, name):
        super().__init__(name)
        
        # Declare parameters
        self.declare_parameter('max_linear_speed', 0.5)       # m/s
        self.declare_parameter('max_angular_speed', 2.0)      # rad/s
        self.declare_parameter('speed_scale', 1.0)            # Overall speed multiplier
        self.declare_parameter('deadzone', 0.2)               # Joystick deadzone
        self.declare_parameter('linear_gear_default', 1.0)    # Starting linear gear
        self.declare_parameter('angular_gear_default', 1.0)   # Starting angular gear
        
        # Get parameters
        self.max_linear_speed = self.get_parameter('max_linear_speed').get_parameter_value().double_value
        self.max_angular_speed = self.get_parameter('max_angular_speed').get_parameter_value().double_value
        self.speed_scale = self.get_parameter('speed_scale').get_parameter_value().double_value
        self.deadzone = self.get_parameter('deadzone').get_parameter_value().double_value
        self.linear_gear = self.get_parameter('linear_gear_default').get_parameter_value().double_value
        self.angular_gear = self.get_parameter('angular_gear_default').get_parameter_value().double_value
        
        # Log loaded parameters
        self.get_logger().info('Joy Teleop Parameters:')
        self.get_logger().info(f'  Max linear speed: {self.max_linear_speed} m/s')
        self.get_logger().info(f'  Max angular speed: {self.max_angular_speed} rad/s')
        self.get_logger().info(f'  Speed scale: {self.speed_scale}')
        self.get_logger().info(f'  Deadzone: {self.deadzone}')
        self.get_logger().info(f'  Linear gear: {self.linear_gear}')
        self.get_logger().info(f'  Angular gear: {self.angular_gear}')
        
        # Create publisher
        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Create subscriber
        self.sub_joy = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        
        # Detect joystick
        joysticks = get_joystick_names()
        self.joystick = joysticks[0] if len(joysticks) != 0 else None
        
        if self.joystick is None:
            self.get_logger().error("No joystick detected! Please connect a controller.")
        else:
            self.get_logger().info(f"Detected joystick: {self.joystick}")
        
        # Button/axis mappings for different controllers
        # Format: [linear_gear_button, angular_gear_button, linear_axis, angular_axis]
        self.controller_config = {
            "Xbox 360 Controller": [9, 10, 1, 3],              # Left stick Y, Right stick X
            "SHANWAN Android Gamepad": [13, 14, 1, 2],         # Left stick Y, Right stick X
            "Xbox One Controller": [9, 10, 1, 3],              # Same as Xbox 360
            "Sony PLAYSTATION(R)3 Controller": [14, 15, 1, 2], # PS3 controller
        }
        
        # Button press state tracking (to detect button press edges)
        self.last_linear_button_state = 0
        self.last_angular_button_state = 0
        
        self.get_logger().info("Joy teleop node started - Ready for input!")

    def joy_callback(self, joy_data):
        """Process joystick input and publish cmd_vel"""
        if not isinstance(joy_data, Joy):
            return
        
        if self.joystick not in self.controller_config:
            self.get_logger().warn(f"Unknown controller: {self.joystick}", throttle_duration_sec=5.0)
            # Use default config for unknown controllers (Xbox 360 layout)
            config = [9, 10, 1, 3]
        else:
            config = self.controller_config[self.joystick]
        
        # Linear gear control (only trigger on button press, not hold)
        linear_button_pressed = joy_data.buttons[config[0]] == 1
        if linear_button_pressed and self.last_linear_button_state == 0:
            if self.linear_gear == 1.0:
                self.linear_gear = 1.0 / 3
            elif self.linear_gear == 1.0 / 3:
                self.linear_gear = 2.0 / 3
            elif self.linear_gear == 2.0 / 3:
                self.linear_gear = 1.0
            self.get_logger().info(f"Linear gear: {self.linear_gear:.2f}")
        self.last_linear_button_state = joy_data.buttons[config[0]]
        
        # Angular gear control (only trigger on button press, not hold)
        angular_button_pressed = joy_data.buttons[config[1]] == 1
        if angular_button_pressed and self.last_angular_button_state == 0:
            if self.angular_gear == 1.0:
                self.angular_gear = 1.0 / 4
            elif self.angular_gear == 1.0 / 4:
                self.angular_gear = 1.0 / 2
            elif self.angular_gear == 1.0 / 2:
                self.angular_gear = 3.0 / 4
            elif self.angular_gear == 3.0 / 4:
                self.angular_gear = 1.0
            self.get_logger().info(f"Angular gear: {self.angular_gear:.2f}")
        self.last_angular_button_state = joy_data.buttons[config[1]]
        
        # Calculate speeds from joystick axes
        linear_axis_value = self.filter_deadzone(joy_data.axes[config[2]])
        angular_axis_value = self.filter_deadzone(joy_data.axes[config[3]])
        
        # Apply speed limits and gears
        linear_speed = linear_axis_value * self.max_linear_speed * self.linear_gear * self.speed_scale
        angular_speed = angular_axis_value * self.max_angular_speed * self.angular_gear * self.speed_scale
        
        # Clamp to maximum speeds (safety check)
        linear_speed = max(-self.max_linear_speed, min(self.max_linear_speed, linear_speed))
        angular_speed = max(-self.max_angular_speed, min(self.max_angular_speed, angular_speed))
        
        # Create and publish Twist message
        twist = Twist()
        twist.linear.x = linear_speed
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = angular_speed
        
        self.pub_cmd_vel.publish(twist)

    def filter_deadzone(self, value):
        """Filter out small joystick values to prevent drift"""
        if abs(value) < self.deadzone:
            return 0.0
        return value


def main(args=None):
    rclpy.init(args=args)
    joy_ctrl = JoyTeleop('joy_teleop')
    
    try:
        rclpy.spin(joy_ctrl)
    except KeyboardInterrupt:
        pass
    finally:
        joy_ctrl.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()