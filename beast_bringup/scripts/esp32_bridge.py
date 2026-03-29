#!/usr/bin/env python3

import serial
import json
import queue
import threading
import rclpy
from rclpy.node import Node
import time
from std_msgs.msg import Header, Float32MultiArray, Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, MagneticField, JointState
from beast_msgs.srv import SetLEDBrightness, UpdateOLED

# Helper class for reading lines from a serial port
class ReadLine:
    def __init__(self, s):
        self.buf = bytearray()
        self.s = s

    def readline(self):
        i = self.buf.find(b"\n")
        if i >= 0:
            r = self.buf[:i+1]
            self.buf = self.buf[i+1:]
            return r
        while True:
            i = max(1, min(512, self.s.in_waiting))
            data = self.s.read(i)
            i = data.find(b"\n")
            if i >= 0:
                r = self.buf + data[:i+1]
                self.buf[0:] = data[i+1:]
                return r
            else:
                self.buf.extend(data)

    def clear_buffer(self):
        self.s.reset_input_buffer()

# Base controller class for managing UART communication
class BaseController:
    def __init__(self, uart_dev_set, baud_set, logger):
        self.logger = logger
        self.ser = serial.Serial(uart_dev_set, baud_set, timeout=1)
        self.rl = ReadLine(self.ser)
        self.command_queue = queue.Queue()
        self.command_thread = threading.Thread(target=self.process_commands, daemon=True)
        self.command_thread.start()
        self.data_buffer = None
        self.base_data = {"T": 1001, "L": 0, "R": 0, "ax": 0, "ay": 0, "az": 0, 
                         "gx": 0, "gy": 0, "gz": 0, "mx": 0, "my": 0, "mz": 0, 
                         "odl": 0, "odr": 0, "v": 0}
    
    def feedback_data(self):
        try:
            line = self.rl.readline().decode('utf-8', errors='ignore')  # Ignore decode errors
            # Skip empty or whitespace-only lines
            if not line.strip():
                return self.base_data
                
            self.data_buffer = json.loads(line)
            self.base_data = self.data_buffer
            return self.base_data
        except json.JSONDecodeError as e:
            # Only log the error without trying to print the corrupted line
            self.logger.debug(f"JSON decode error, clearing buffer")
            self.rl.clear_buffer()
            return self.base_data
        except Exception as e:
            self.logger.debug(f"Serial read error, clearing buffer")
            self.rl.clear_buffer()
            return self.base_data

    def send_command(self, data):
        self.command_queue.put(data)

    def process_commands(self):
        while True:
            data = self.command_queue.get()
            self.ser.write((json.dumps(data) + '\n').encode("utf-8"))

    def base_json_ctrl(self, input_json):
        self.send_command(input_json)

    def close(self):
        self.ser.close()

# Unified ESP32 bridge node - handles all ESP32 communication
class ESP32Bridge(Node):
    def __init__(self):
        super().__init__('esp32_bridge')
        
        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyAMA0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('feedback_rate', 1000.0)  # Hz
        self.declare_parameter('min_angular_threshold', 0.2)  # Minimum angular velocity when stationary
        self.declare_parameter('low_voltage_threshold', 9.0)
        self.declare_parameter('warning_cooldown', 5.0)  # seconds between warnings
        self.declare_parameter('cmd_vel_timeout', 0.5)  # seconds - stop if no cmd_vel received
        
        # Get parameters
        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        feedback_rate = self.get_parameter('feedback_rate').get_parameter_value().double_value
        self.min_angular_threshold = self.get_parameter('min_angular_threshold').get_parameter_value().double_value
        self.low_voltage_threshold = self.get_parameter('low_voltage_threshold').get_parameter_value().double_value
        self.warning_cooldown = self.get_parameter('warning_cooldown').get_parameter_value().double_value
        self.cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').get_parameter_value().double_value
        
        # Log loaded parameters
        self.get_logger().info('ESP32 Bridge Parameters:')
        self.get_logger().info(f'  Serial port: {serial_port}')
        self.get_logger().info(f'  Baud rate: {baud_rate}')
        self.get_logger().info(f'  Feedback rate: {feedback_rate} Hz')
        self.get_logger().info(f'  Min angular threshold: {self.min_angular_threshold} rad/s')
        self.get_logger().info(f'  Low voltage threshold: {self.low_voltage_threshold}V')
        self.get_logger().info(f'  Cmd_vel timeout: {self.cmd_vel_timeout}s')
        
        # Initialize the base controller with the UART port and baud rate
        self.base_controller = BaseController(serial_port, baud_rate, self.get_logger())
        
        # Command timeout tracking
        self.last_cmd_vel_time = time.time()
        self.motors_stopped = True  # Track if we've already sent stop command
        
        # Publishers for sensor data
        self.imu_data_raw_publisher_ = self.create_publisher(Imu, "imu/data_raw", 100)
        self.imu_mag_publisher_ = self.create_publisher(MagneticField, "imu/mag", 100)
        self.odom_publisher_ = self.create_publisher(Float32MultiArray, "odom/odom_raw", 100)
        self.voltage_publisher_ = self.create_publisher(Float32, "voltage", 50)
        
        # Subscribers for command data
        self.cmd_vel_sub_ = self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 10)
        self.joint_states_sub = self.create_subscription(JointState, 'ugv/joint_states', self.joint_states_callback, 10)
        self.led_service = self.create_service(SetLEDBrightness, 'ugv/set_headlights', self.led_service_callback)
        
        # Timer to periodically read sensor feedback
        timer_period = 1.0 / feedback_rate
        self.feedback_timer = self.create_timer(timer_period, self.feedback_loop)
        
        # Timer to check for command timeout (run at 20 Hz)
        self.watchdog_timer = self.create_timer(0.05, self.watchdog_check)
        
        # Low battery warning state
        self.last_warning_time = 0

        #OLED service
        self.oled_service = self.create_service(UpdateOLED, 'update_oled', self.oled_service_callback)
        
        self.get_logger().info('ESP32 Bridge started successfully')

    # ========== WATCHDOG/SAFETY METHODS ==========
    
    def watchdog_check(self):
        """Check if cmd_vel timeout has occurred and stop motors if needed"""
        current_time = time.time()
        time_since_last_cmd = current_time - self.last_cmd_vel_time
        
        if time_since_last_cmd > self.cmd_vel_timeout and not self.motors_stopped:
            # Timeout occurred - send stop command
            self.get_logger().warn(f'Cmd_vel timeout ({time_since_last_cmd:.2f}s) - stopping motors')
            self.send_stop_command()
            self.motors_stopped = True
    
    def send_stop_command(self):
        """Send a stop command to the ESP32"""
        data = {'T': '13', 'X': 0.0, 'Z': 0.0}
        self.base_controller.send_command(data)

    # ========== FEEDBACK/PUBLISHING METHODS ==========
    
    def feedback_loop(self):
        self.base_controller.feedback_data()
        if self.base_controller.base_data.get("T") == 1001:
            self.publish_imu_data_raw()
            self.publish_imu_mag()
            self.publish_odom_raw()
            self.publish_voltage()

    def publish_imu_data_raw(self):
        msg = Imu()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_imu_link"
        imu_raw_data = self.base_controller.base_data

        msg.linear_acceleration.x = 9.8 * float(imu_raw_data["ax"]) / 8192
        msg.linear_acceleration.y = 9.8 * float(imu_raw_data["ay"]) / 8192
        msg.linear_acceleration.z = 9.8 * float(imu_raw_data["az"]) / 8192
        
        msg.angular_velocity.x = 3.1415926 * float(imu_raw_data["gx"]) / (16.4 * 180)
        msg.angular_velocity.y = 3.1415926 * float(imu_raw_data["gy"]) / (16.4 * 180)
        msg.angular_velocity.z = 3.1415926 * float(imu_raw_data["gz"]) / (16.4 * 180)
              
        self.imu_data_raw_publisher_.publish(msg)
        
    def publish_imu_mag(self):
        msg = MagneticField()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_imu_link"
        imu_raw_data = self.base_controller.base_data

        msg.magnetic_field.x = float(imu_raw_data["mx"]) * 0.15
        msg.magnetic_field.y = float(imu_raw_data["my"]) * 0.15
        msg.magnetic_field.z = float(imu_raw_data["mz"]) * 0.15
              
        self.imu_mag_publisher_.publish(msg)

    def publish_odom_raw(self):
        odom_raw_data = self.base_controller.base_data
        array = [odom_raw_data["odl"]/100, odom_raw_data["odr"]/100]
        msg = Float32MultiArray(data=array)
        self.odom_publisher_.publish(msg)

    def publish_voltage(self):
        voltage_data = self.base_controller.base_data
        voltage_value = float(voltage_data["v"]) / 100
        msg = Float32()
        msg.data = voltage_value
        self.voltage_publisher_.publish(msg)
        
        # Check for low battery and trigger warning
        self.check_low_battery(voltage_value)

    def check_low_battery(self, voltage_value):
        current_time = time.time()
        if 0.1 < voltage_value < self.low_voltage_threshold:
            if (current_time - self.last_warning_time) > self.warning_cooldown:
                self.get_logger().warn(f'Low battery detected: {voltage_value}V')      
                self.last_warning_time = current_time

    # ========== COMMAND/SUBSCRIPTION CALLBACKS ==========

    def cmd_vel_callback(self, msg):
        # Update last command time
        self.last_cmd_vel_time = time.time()
        self.motors_stopped = False  # Reset stopped flag when new command arrives
        
        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z

        # Apply minimum threshold to angular velocity if linear velocity is zero
        if linear_velocity == 0:
            if 0 < angular_velocity < self.min_angular_threshold:
                angular_velocity = self.min_angular_threshold
            elif -self.min_angular_threshold < angular_velocity < 0:
                angular_velocity = -self.min_angular_threshold

        # Send velocity command to ESP32
        data = {'T': '13', 'X': linear_velocity, 'Z': angular_velocity}
        self.base_controller.send_command(data)

    def joint_states_callback(self, msg):
        # Extract joint positions and convert to degrees
        name = msg.name
        position = msg.position

        x_rad = position[name.index('pt_base_link_to_pt_link1')]
        y_rad = position[name.index('pt_link1_to_pt_link2')]

        x_degree = (180 * x_rad) / 3.1415926
        y_degree = (180 * y_rad) / 3.1415926

        # Send joint command to ESP32
        joint_data = {
            'T': 134, 
            'X': x_degree, 
            'Y': y_degree, 
            "SX": 600,
            "SY": 600,
        }
        self.base_controller.send_command(joint_data)

    def led_service_callback(self, request, response):
        """Service callback to control headlight brightness"""
        brightness = request.brightness
        
        # Clamp brightness to valid range
        if brightness < 0.0:
            brightness = 0.0
        elif brightness > 255.0:
            brightness = 255.0
        
        # Send LED control command to ESP32
        led_ctrl_data = {
            'T': 132, 
            "IO4": brightness,
            "IO5": brightness,
        }
        self.base_controller.send_command(led_ctrl_data)
        
        # Set response
        response.success = True
        response.message = f"Headlights set to brightness: {brightness}"
        
        self.get_logger().debug(f"Headlights brightness: {brightness}")
        
        return response

    def oled_service_callback(self, request, response):
        data = {'T': 3, 'lineNum': int(request.line_num), 'Text': request.text}
        self.base_controller.send_command(data)
        response.success = True
        response.message = 'OK'
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ESP32Bridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Send stop command before shutting down
        node.send_stop_command()
        node.base_controller.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()