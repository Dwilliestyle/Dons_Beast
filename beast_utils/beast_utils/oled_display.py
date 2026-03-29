#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from beast_msgs.srv import UpdateOLED
import socket
import time
from datetime import datetime


class OLEDDisplay(Node):
    def __init__(self):
        super().__init__('oled_display')
        
        # Declare parameters
        self.declare_parameter('update_rate', 1.0)  # Hz
        self.declare_parameter('cmd_timeout', 0.5)  # seconds
        self.declare_parameter('oled_delay', 0.15)  # seconds between OLED updates
        
        # Get parameters
        update_rate = self.get_parameter('update_rate').get_parameter_value().double_value
        self.cmd_timeout = self.get_parameter('cmd_timeout').get_parameter_value().double_value
        self.oled_delay = self.get_parameter('oled_delay').get_parameter_value().double_value
        
        # Create service client for OLED updates
        self.oled_client = self.create_client(UpdateOLED, 'update_oled')
        
        # Wait for service to be available
        self.get_logger().info('Waiting for update_oled service...')
        while not self.oled_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        self.get_logger().info('Connected to update_oled service')
        
        # Display data
        self.ip_address = self.get_ip_address()
        self.battery_voltage = None
        self.robot_status = "Stopped"
        self.last_cmd_time = time.time()
        
        # Create subscriptions
        self.voltage_sub = self.create_subscription(
            Float32,
            'voltage',
            self.voltage_callback,
            10
        )
        
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Create timer to update display
        timer_period = 1.0 / update_rate
        self.display_timer = self.create_timer(timer_period, self.update_display)
        self.status_timer = self.create_timer(0.1, self.check_status)
        
        self.get_logger().info(f'OLED display node started')
        self.get_logger().info(f'  Update rate: {update_rate} Hz')
        self.get_logger().info(f'  OLED delay: {self.oled_delay}s')
        self.get_logger().info(f'  IP Address: {self.ip_address}')
    
    def get_ip_address(self):
        """Get the robot's IP address"""
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except Exception:
            return "No Network"
    
    def voltage_callback(self, msg):
        """Receive voltage updates from ESP32 bridge"""
        self.battery_voltage = msg.data
    
    def cmd_vel_callback(self, msg):
        """Monitor cmd_vel to determine robot status"""
        self.last_cmd_time = time.time()
        
        linear = abs(msg.linear.x)
        angular = abs(msg.angular.z)
        
        # Determine robot status
        if linear < 0.01 and angular < 0.01:
            self.robot_status = "Stopped"
        elif angular > 0.1:
            self.robot_status = "Turning"
        else:
            self.robot_status = "Moving"
    
    def check_status(self):
        """Check if robot has timed out (no recent commands)"""
        if time.time() - self.last_cmd_time > self.cmd_timeout:
            if self.robot_status != "Stopped":
                self.robot_status = "Stopped"
    
    def send_oled_update(self, line_num, text):
        """Call OLED update service (async - fire and forget)"""
        try:
            request = UpdateOLED.Request()
            request.line_num = line_num
            request.text = text
            
            # Async call - don't wait for response
            future = self.oled_client.call_async(request)
            self.get_logger().debug(f'Sent OLED update: line {line_num} = "{text}"')
        except Exception as e:
            self.get_logger().error(f'Failed to call OLED service: {e}')
    
    def update_display(self):
        current_time = datetime.now().strftime('%H:%M:%S')
        self.send_oled_update(0, f"Time: {current_time}")
        time.sleep(self.oled_delay)

        self.send_oled_update(1, f"Status: {self.robot_status}")
        time.sleep(self.oled_delay)

        if self.battery_voltage is not None:
            if self.battery_voltage < 7.0:
                self.send_oled_update(2, f"BATT!: {self.battery_voltage:.2f}V!")
            elif self.battery_voltage < 7.5:
                self.send_oled_update(2, f"Batt!: {self.battery_voltage:.2f}V")
            else:
                self.send_oled_update(2, f"Batt: {self.battery_voltage:.2f}V")
        else:
            self.send_oled_update(2, "Batt: --.--V")
        time.sleep(self.oled_delay)

        self.send_oled_update(3, f"IP:{self.ip_address}")


def main(args=None):
    rclpy.init(args=args)
    oled_display = OLEDDisplay()
    
    try:
        rclpy.spin(oled_display)
    except KeyboardInterrupt:
        pass
    finally:
        oled_display.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()