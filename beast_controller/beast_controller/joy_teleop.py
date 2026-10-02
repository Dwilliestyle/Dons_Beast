#!/usr/bin/env python3
# encoding: utf-8
"""
Joystick teleop for UGV Beast
Subscribes to /joy, publishes cmd_vel, and controls headlight brightness
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from beast_msgs.srv import SetLEDBrightness
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
        self.declare_parameter('max_linear_speed', 0.5)
        self.declare_parameter('max_angular_speed', 2.0)
        self.declare_parameter('speed_scale', 1.0)
        self.declare_parameter('deadzone', 0.2)
        self.declare_parameter('linear_axis', 1)
        self.declare_parameter('angular_axis', 3)
        self.declare_parameter('headlight_up_button', 0)
        self.declare_parameter('headlight_down_button', 1)
        self.declare_parameter('headlight_step', 64.0)

        # Get parameters
        self.max_linear_speed = self.get_parameter('max_linear_speed').get_parameter_value().double_value
        self.max_angular_speed = self.get_parameter('max_angular_speed').get_parameter_value().double_value
        self.speed_scale = self.get_parameter('speed_scale').get_parameter_value().double_value
        self.deadzone = self.get_parameter('deadzone').get_parameter_value().double_value
        self.linear_axis = self.get_parameter('linear_axis').get_parameter_value().integer_value
        self.angular_axis = self.get_parameter('angular_axis').get_parameter_value().integer_value
        self.headlight_up_btn = self.get_parameter('headlight_up_button').get_parameter_value().integer_value
        self.headlight_down_btn = self.get_parameter('headlight_down_button').get_parameter_value().integer_value
        self.headlight_step = self.get_parameter('headlight_step').get_parameter_value().double_value

        # Log parameters
        self.get_logger().info('Joy Teleop Parameters:')
        self.get_logger().info(f'  Max linear speed: {self.max_linear_speed} m/s')
        self.get_logger().info(f'  Max angular speed: {self.max_angular_speed} rad/s')
        self.get_logger().info(f'  Speed scale: {self.speed_scale}')
        self.get_logger().info(f'  Deadzone: {self.deadzone}')
        self.get_logger().info(f'  Linear axis: {self.linear_axis}')
        self.get_logger().info(f'  Angular axis: {self.angular_axis}')
        self.get_logger().info(f'  Headlight up button: {self.headlight_up_btn}')
        self.get_logger().info(f'  Headlight down button: {self.headlight_down_btn}')
        self.get_logger().info(f'  Headlight step: {self.headlight_step}')

        # Publisher and subscriber
        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        self.sub_joy = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

        # Headlight service clients - one per light channel on the ESP32
        self.headlight_clients = [
            self.create_client(SetLEDBrightness, '/ugv/set_headlights'),      # IO4
            self.create_client(SetLEDBrightness, '/ugv/set_headlights_io5'),  # IO5
        ]
        self.headlight_brightness = 0.0

        # Button state tracking (so one press = one step)
        self.last_light_up_state = 0
        self.last_light_down_state = 0

        # Detect joystick (name is only used for logging)
        joysticks = get_joystick_names()
        self.joystick = joysticks[0] if len(joysticks) != 0 else None

        if self.joystick is None:
            self.get_logger().error("No joystick detected! Please connect a controller.")
        else:
            self.get_logger().info(f"Detected joystick: {self.joystick}")

        self.get_logger().info("Joy teleop node started - Ready for input!")

    def set_headlights(self, brightness):
        """Set both light channels (IO4 and IO5) to the same brightness"""
        for client in self.headlight_clients:
            if not client.service_is_ready():
                self.get_logger().warn(f"Headlight service not available: {client.srv_name}")
                continue
            req = SetLEDBrightness.Request()
            req.brightness = brightness
            client.call_async(req)
        self.get_logger().info(f"Headlights: {brightness:.0f}/255 ({brightness / 255 * 100:.0f}%)")

    def joy_callback(self, joy_data):
        """Process joystick input and publish cmd_vel"""
        if not isinstance(joy_data, Joy):
            return

        # --- Headlight brightness up ---
        light_up_pressed = joy_data.buttons[self.headlight_up_btn] == 1
        if light_up_pressed and self.last_light_up_state == 0:
            self.headlight_brightness = min(255.0, self.headlight_brightness + self.headlight_step)
            self.set_headlights(self.headlight_brightness)
        self.last_light_up_state = joy_data.buttons[self.headlight_up_btn]

        # --- Headlight brightness down ---
        light_down_pressed = joy_data.buttons[self.headlight_down_btn] == 1
        if light_down_pressed and self.last_light_down_state == 0:
            self.headlight_brightness = max(0.0, self.headlight_brightness - self.headlight_step)
            self.set_headlights(self.headlight_brightness)
        self.last_light_down_state = joy_data.buttons[self.headlight_down_btn]

        # --- Calculate and publish cmd_vel ---
        linear_speed = self.filter_deadzone(joy_data.axes[self.linear_axis]) * self.max_linear_speed * self.speed_scale
        angular_speed = self.filter_deadzone(joy_data.axes[self.angular_axis]) * self.max_angular_speed * self.speed_scale

        linear_speed = max(-self.max_linear_speed, min(self.max_linear_speed, linear_speed))
        angular_speed = max(-self.max_angular_speed, min(self.max_angular_speed, angular_speed))

        twist = Twist()
        twist.linear.x = linear_speed
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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()