#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import subprocess

try:
    import smbus
    SMBUS_AVAILABLE = True
except ImportError:
    SMBUS_AVAILABLE = False


class BatteryMonitor(Node):
    def __init__(self):
        super().__init__('battery_monitor')
        
        # Declare parameters
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('ina219_address', 0x42)
        self.declare_parameter('low_voltage_threshold', 10.0)
        self.declare_parameter('critical_voltage_threshold', 9.5)
        self.declare_parameter('min_voltage', 6.0)
        self.declare_parameter('max_voltage', 13.0)
        self.declare_parameter('update_rate', 5.0)  # Hz
        
        # Get parameters
        i2c_bus = self.get_parameter('i2c_bus').get_parameter_value().integer_value
        self.ina219_addr = self.get_parameter('ina219_address').get_parameter_value().integer_value
        self.low_voltage = self.get_parameter('low_voltage_threshold').get_parameter_value().double_value
        self.critical_voltage = self.get_parameter('critical_voltage_threshold').get_parameter_value().double_value
        self.min_voltage = self.get_parameter('min_voltage').get_parameter_value().double_value
        self.max_voltage = self.get_parameter('max_voltage').get_parameter_value().double_value
        update_rate = self.get_parameter('update_rate').get_parameter_value().double_value
        
        # Battery warning flags (to only warn once per threshold)
        self.low_battery_warned = False
        self.critical_battery_warned = False
        
        # Initialize I2C
        self.i2c_bus = None
        if SMBUS_AVAILABLE:
            try:
                self.i2c_bus = smbus.SMBus(i2c_bus)
                self.get_logger().info(f'Initialized I2C bus {i2c_bus} for INA219 at address 0x{self.ina219_addr:02X}')
            except Exception as e:
                self.get_logger().error(f'Could not initialize I2C: {e}')
                self.i2c_bus = None
        else:
            self.get_logger().error('smbus library not available - battery monitoring disabled')
        
        # Create publisher
        self.voltage_pub = self.create_publisher(Float32, 'battery_voltage', 10)
        
        # Create timer
        timer_period = 1.0 / update_rate
        self.timer = self.create_timer(timer_period, self.read_battery)
        
        self.get_logger().info('Battery monitor node started')
    
    def play_audio_warning(self, message):
        """Play an audible warning using espeak"""
        try:
            subprocess.Popen(
                ['espeak', message],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
        except Exception as e:
            self.get_logger().debug(f'Could not play audio warning: {e}')
    
    def read_battery(self):
        """Read battery voltage from INA219 sensor"""
        if not self.i2c_bus:
            return
        
        try:
            REG_BUSVOLTAGE = 0x02
            
            # Read the bus voltage register
            raw = self.i2c_bus.read_word_data(self.ina219_addr, REG_BUSVOLTAGE)
            
            # Swap bytes (INA219 is big-endian)
            raw = ((raw & 0xFF) << 8) | ((raw & 0xFF00) >> 8)
            
            # Extract voltage (bits 3-15, LSB = 4mV)
            voltage = (raw >> 3) * 0.004
            
            # Sanity check - typical range for robot battery
            if self.min_voltage < voltage < self.max_voltage:
                # Publish voltage
                voltage_msg = Float32()
                voltage_msg.data = voltage
                self.voltage_pub.publish(voltage_msg)
                
                self.get_logger().debug(f'Battery voltage: {voltage:.2f}V')
                
                # Check battery levels and issue warnings
                if voltage < self.critical_voltage:
                    self.get_logger().error(f'🔴 CRITICAL BATTERY: {voltage:.2f}V - Stop and recharge immediately!')
                    if not self.critical_battery_warned:
                        self.play_audio_warning('Critical battery. Stop and recharge immediately')
                        self.critical_battery_warned = True
                        self.low_battery_warned = True  # Set both flags
                        
                elif voltage < self.low_voltage:
                    self.get_logger().warn(f'⚠️  LOW BATTERY: {voltage:.2f}V - Consider recharging soon!')
                    if not self.low_battery_warned:
                        self.play_audio_warning('Low battery warning')
                        self.low_battery_warned = True
                        
                else:
                    # Reset warning flags when battery is good
                    self.low_battery_warned = False
                    self.critical_battery_warned = False
            else:
                self.get_logger().debug(f'Voltage reading out of range: {voltage:.2f}V')
                
        except OSError as e:
            # Resource temporarily unavailable - normal if ESP32 is also accessing I2C
            if e.errno == 11:
                self.get_logger().debug('I2C busy, will retry next cycle')
            else:
                self.get_logger().debug(f'I2C error: {e}')
                
        except Exception as e:
            self.get_logger().debug(f'Error reading battery: {e}')


def main(args=None):
    rclpy.init(args=args)
    battery_monitor = BatteryMonitor()
    
    try:
        rclpy.spin(battery_monitor)
    except KeyboardInterrupt:
        pass
    finally:
        battery_monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()