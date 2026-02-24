#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray
from tf2_ros import TransformBroadcaster
import math


class OdomPublisher(Node):
    def __init__(self):
        super().__init__('odom_publisher')
        
        # Declare parameters
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_footprint_frame', 'base_footprint')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('pub_odom_tf', True)
        self.declare_parameter('track_separation', 0.143)  # meters - track width
        self.declare_parameter('publish_rate', 10.0)      # Hz
        self.declare_parameter('use_imu_heading', False)  # Use IMU yaw instead of wheel odometry
        
        # Get parameters
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.base_footprint_frame = self.get_parameter('base_footprint_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.pub_odom_tf = self.get_parameter('pub_odom_tf').get_parameter_value().bool_value
        self.track_separation = self.get_parameter('track_separation').get_parameter_value().double_value
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.use_imu_heading = self.get_parameter('use_imu_heading').get_parameter_value().bool_value
        
        # Log loaded parameters
        self.get_logger().info('Odometry Publisher Parameters:')
        self.get_logger().info(f'  Odom frame: {self.odom_frame}')
        self.get_logger().info(f'  Base footprint frame: {self.base_footprint_frame}')
        self.get_logger().info(f'  Base frame: {self.base_frame}')
        self.get_logger().info(f'  Publish TF: {self.pub_odom_tf}')
        self.get_logger().info(f'  Wheel separation: {self.track_separation}m')
        self.get_logger().info(f'  Publish rate: {publish_rate}Hz')
        self.get_logger().info(f'  Use IMU heading: {self.use_imu_heading}')
        
        # Initialize variables
        self.x_pos = 0.0
        self.y_pos = 0.0
        self.yaw = 0.0
        self.odom_yaw = 0.0
        self.imu_yaw = 0.0
        
        self.vx = 0.0
        self.vw = 0.0
        
        self.pre_odl = 0.0
        self.pre_odr = 0.0
        
        self.init_odl = 0.0
        self.init_odr = 0.0
        self.is_initialized = False
        
        self.last_time = self.get_clock().now()
        
        # Covariance matrices for odometry
        # Used when robot is moving
        self.ODOM_POSE_COVARIANCE = [
            1e-3, 0, 0, 0, 0, 0,
            0, 1e-3, 0, 0, 0, 0,
            0, 0, 1e6, 0, 0, 0,
            0, 0, 0, 1e6, 0, 0,
            0, 0, 0, 0, 1e6, 0,
            0, 0, 0, 0, 0, 1e3
        ]
        
        self.ODOM_TWIST_COVARIANCE = [
            1e-3, 0, 0, 0, 0, 0,
            0, 1e-3, 0, 0, 0, 0,
            0, 0, 1e6, 0, 0, 0,
            0, 0, 0, 1e6, 0, 0,
            0, 0, 0, 0, 1e6, 0,
            0, 0, 0, 0, 0, 1e3
        ]
        
        # Used when robot is stationary
        self.ODOM_POSE_COVARIANCE2 = [
            1e-9, 0, 0, 0, 0, 0,
            0, 1e-3, 1e-9, 0, 0, 0,
            0, 0, 1e6, 0, 0, 0,
            0, 0, 0, 1e6, 0, 0,
            0, 0, 0, 0, 1e6, 0,
            0, 0, 0, 0, 0, 1e-9
        ]
        
        self.ODOM_TWIST_COVARIANCE2 = [
            1e-9, 0, 0, 0, 0, 0,
            0, 1e-3, 1e-9, 0, 0, 0,
            0, 0, 1e6, 0, 0, 0,
            0, 0, 0, 1e6, 0, 0,
            0, 0, 0, 0, 1e6, 0,
            0, 0, 0, 0, 0, 1e-9
        ]
        
        # Create transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Create subscriptions
        self.imu_sub = self.create_subscription(
            Imu,
            'imu/data',
            self.imu_callback,
            10
        )
        
        self.odom_raw_sub = self.create_subscription(
            Float32MultiArray,
            'odom/odom_raw',
            self.odom_raw_callback,
            50
        )
        
        # Create publisher
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        
        # Create timer to publish odometry
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.publish_odom)
        
        self.get_logger().info('Odometry publisher node started')
    
    def imu_callback(self, msg):
        """Handle IMU data to get robot orientation"""
        # Extract quaternion from IMU
        q0 = msg.orientation.w
        q1 = msg.orientation.x
        q2 = msg.orientation.y
        q3 = msg.orientation.z
        
        # Convert quaternion to yaw angle
        siny_cosp = 2.0 * (q0 * q3 + q1 * q2)
        cosy_cosp = 1.0 - 2.0 * (q2 * q2 + q3 * q3)
        self.imu_yaw = math.atan2(siny_cosp, cosy_cosp)
    
    def odom_raw_callback(self, msg):
        """Handle raw odometry data from wheel encoders"""
        current_time = self.get_clock().now()
        
        # Get current encoder readings
        now_odl = msg.data[0]  # Left wheel encoder
        now_odr = msg.data[1]  # Right wheel encoder
        
        # Initialize on first reading
        if not self.is_initialized:
            self.init_odl = now_odl
            self.init_odr = now_odr
            self.is_initialized = True
            self.last_time = current_time
            return
        
        # Remove initial offset
        now_odl -= self.init_odl
        now_odr -= self.init_odr
        
        # Calculate time difference
        dt = (current_time - self.last_time).nanoseconds / 1e9  # Convert to seconds
        self.last_time = current_time
        
        if dt == 0:
            return
        
        # Calculate distance traveled by each wheel since last update
        dleft = now_odl - self.pre_odl
        dright = now_odr - self.pre_odr
        
        # Update previous values
        self.pre_odl = now_odl
        self.pre_odr = now_odr
        
        # Calculate robot motion
        # Average distance traveled
        dxy_ave = (dright + dleft) / 2.0
        
        # Change in heading angle
        dth = (dright - dleft) / self.track_separation
        
        # Calculate velocities
        self.vx = dxy_ave / dt  # Linear velocity
        self.vw = dth / dt      # Angular velocity
        
        # Update position if robot moved forward/backward
        if dxy_ave != 0:
            dx = math.cos(dth / 2.0) * dxy_ave
            dy = math.sin(dth / 2.0) * dxy_ave
            
            # Update global position
            self.x_pos += (math.cos(self.yaw) * dx - math.sin(self.yaw) * dy)
            self.y_pos += (math.sin(self.yaw) * dx + math.cos(self.yaw) * dy)
        
        # Update heading if robot rotated
        if dth != 0:
            self.odom_yaw += dth
        
        # Choose heading source based on parameter
        if self.use_imu_heading:
            self.yaw = self.imu_yaw
        else:
            self.yaw = self.odom_yaw
    
    def publish_odom(self):
        """Publish odometry message and optionally transform"""
        current_time = self.get_clock().now()
        
        # Create odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_footprint_frame
        
        # Set position
        odom.pose.pose.position.x = self.x_pos
        odom.pose.pose.position.y = self.y_pos
        odom.pose.pose.position.z = 0.0
        
        # Set orientation (convert yaw to quaternion)
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        
        # Set velocities
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = self.vw
        
        # Set covariance based on whether robot is moving or stationary
        if self.vx == 0 and self.vw == 0:
            # Robot is stationary - use tighter covariance
            odom.pose.covariance = self.ODOM_POSE_COVARIANCE2
            odom.twist.covariance = self.ODOM_TWIST_COVARIANCE2
        else:
            # Robot is moving - use normal covariance
            odom.pose.covariance = self.ODOM_POSE_COVARIANCE
            odom.twist.covariance = self.ODOM_TWIST_COVARIANCE
        
        # Publish odometry message
        self.odom_pub.publish(odom)
        
        # Optionally publish transform
        if self.pub_odom_tf:
            t = TransformStamped()
            t.header.stamp = current_time.to_msg()
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_footprint_frame
            
            # Set translation
            t.transform.translation.x = self.x_pos
            t.transform.translation.y = self.y_pos
            t.transform.translation.z = 0.0
            
            # Set rotation
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = math.sin(self.yaw / 2.0)
            t.transform.rotation.w = math.cos(self.yaw / 2.0)
            
            # Broadcast transform
            self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    odom_publisher = OdomPublisher()
    
    try:
        rclpy.spin(odom_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        odom_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()