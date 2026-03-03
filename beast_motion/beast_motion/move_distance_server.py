#!/usr/bin/env python3
"""
move_distance_server.py - ROS2 Action Server for distance and turn moves.

Subscribes to /odom to track position, publishes to /cmd_vel to move.
Accepts goals with distance (meters) and/or turn_degrees.

Action: beast_msgs/action/MoveDistance

Usage examples:
  - Move forward 1 meter:     distance=1.0, turn_degrees=0.0
  - Move backward 0.5m:       distance=-0.5, turn_degrees=0.0
  - Turn left 90 degrees:     distance=0.0, turn_degrees=90.0
  - Turn right 45 degrees:    distance=0.0, turn_degrees=-45.0
  - Move forward 1m + turn:   distance=1.0, turn_degrees=90.0 (sequential)
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from beast_msgs.action import MoveDistance


# ── Tuning constants ──────────────────────────────────────────────────────────
DEFAULT_LINEAR_SPEED  = 0.20   # m/s  – safe default forward speed
DEFAULT_ANGULAR_SPEED = 30.0   # deg/s – safe default turn speed

# How close we need to get before calling the move "done"
DISTANCE_TOLERANCE = 0.02   # meters  (about 3/4 inch)
ANGLE_TOLERANCE    = 2.0    # degrees

# Slow-down zone: start decelerating when this close to the goal
DISTANCE_SLOWDOWN  = 0.15   # meters
ANGLE_SLOWDOWN     = 20.0   # degrees

# Minimum speeds to keep the robot actually moving while decelerating
MIN_LINEAR_SPEED   = 0.05   # m/s
MIN_ANGULAR_SPEED  = 8.0    # deg/s

# Safety timeout (seconds) – abort if goal takes longer than this
MOVE_TIMEOUT       = 30.0

# Heading hold during straight moves
# How aggressively to correct drift (tune this if correction is too twitchy or too weak)
HEADING_KP         = 2.0   # proportional gain: correction = HEADING_KP * yaw_error_radians
MAX_HEADING_CORRECTION = 0.3  # rad/s – cap so heading hold doesn't overpower forward motion
# ─────────────────────────────────────────────────────────────────────────────


def yaw_from_quaternion(q):
    """Extract yaw (rotation around Z) from a geometry_msgs quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def angle_diff(a, b):
    """Shortest signed angular difference from b to a, in radians."""
    d = a - b
    while d > math.pi:
        d -= 2.0 * math.pi
    while d < -math.pi:
        d += 2.0 * math.pi
    return d


class MoveDistanceServer(Node):

    def __init__(self):
        super().__init__('move_distance_server')

        # Current odometry state
        self.current_x   = 0.0
        self.current_y   = 0.0
        self.current_yaw = 0.0   # radians
        self.odom_ready  = False

        # Use ReentrantCallbackGroup so the action and odom callbacks
        # can run concurrently in the MultiThreadedExecutor.
        self.cb_group = ReentrantCallbackGroup()

        # Odometry subscriber
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self._odom_callback,
            10,
            callback_group=self.cb_group,
        )

        # cmd_vel publisher
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Action server
        self._action_server = ActionServer(
            self,
            MoveDistance,
            'move_distance',
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self.cb_group,
        )

        self.get_logger().info('MoveDistance action server ready.')

    # ── Odometry ──────────────────────────────────────────────────────────────

    def _odom_callback(self, msg: Odometry):
        self.current_x   = msg.pose.pose.position.x
        self.current_y   = msg.pose.pose.position.y
        self.current_yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.odom_ready  = True

    # ── Action callbacks ──────────────────────────────────────────────────────

    def _goal_callback(self, goal_request):
        self.get_logger().info(
            f'Received goal: distance={goal_request.distance:.3f}m  '
            f'turn={goal_request.turn_degrees:.1f}°'
        )
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        self.get_logger().info('Cancel requested.')
        return CancelResponse.ACCEPT

    # ── Main execute ──────────────────────────────────────────────────────────

    def _execute_callback(self, goal_handle):
        goal = goal_handle.request
        feedback = MoveDistance.Feedback()
        result   = MoveDistance.Result()

        # Resolve speeds from goal, falling back to defaults
        linear_speed  = goal.linear_speed  if goal.linear_speed  > 0.0 else DEFAULT_LINEAR_SPEED
        angular_speed = goal.angular_speed if goal.angular_speed > 0.0 else DEFAULT_ANGULAR_SPEED
        angular_speed_rad = math.radians(angular_speed)

        # ── Wait for first odom message ───────────────────────────────────────
        wait_start = self.get_clock().now()
        while not self.odom_ready:
            if (self.get_clock().now() - wait_start).nanoseconds / 1e9 > 5.0:
                self.get_logger().error('Timed out waiting for /odom.')
                self._stop()
                result.success = False
                result.message = 'No odometry received'
                goal_handle.abort()
                return result
            rclpy.spin_once(self, timeout_sec=0.05)

        # ── Phase 1: linear move (if requested) ───────────────────────────────
        distance_traveled = 0.0
        if abs(goal.distance) > DISTANCE_TOLERANCE:
            distance_traveled = self._run_linear(
                goal_handle, feedback, goal.distance, linear_speed
            )
            if goal_handle.is_cancel_requested:
                self._stop()
                result.success        = False
                result.distance_traveled = distance_traveled
                result.angle_turned   = 0.0
                result.message        = 'Cancelled during linear move'
                goal_handle.canceled()
                return result

        # ── Phase 2: turn (if requested) ─────────────────────────────────────
        angle_turned = 0.0
        if abs(goal.turn_degrees) > ANGLE_TOLERANCE:
            angle_turned = self._run_turn(
                goal_handle, feedback, goal.turn_degrees, angular_speed_rad
            )
            if goal_handle.is_cancel_requested:
                self._stop()
                result.success        = False
                result.distance_traveled = distance_traveled
                result.angle_turned   = angle_turned
                result.message        = 'Cancelled during turn'
                goal_handle.canceled()
                return result

        self._stop()
        result.success           = True
        result.distance_traveled = distance_traveled
        result.angle_turned      = angle_turned
        result.message           = 'Goal reached'
        goal_handle.succeed()
        self.get_logger().info(
            f'Done. Traveled {distance_traveled:.3f}m, turned {angle_turned:.1f}°'
        )
        return result

    # ── Linear move phase ─────────────────────────────────────────────────────

    def _run_linear(self, goal_handle, feedback, target_distance, linear_speed):
        """Drive forward or backward target_distance meters. Returns distance traveled.
        
        Uses heading hold to correct drift — records starting yaw and applies a
        small proportional angular correction throughout the move to keep straight.
        """
        start_x   = self.current_x
        start_y   = self.current_y
        start_yaw = self.current_yaw   # target heading — stay on this bearing
        sign      = 1.0 if target_distance >= 0 else -1.0
        target    = abs(target_distance)

        start_time = self.get_clock().now()
        rate = self.create_rate(20)  # 20 Hz control loop

        while rclpy.ok():
            # Check for cancel
            if goal_handle.is_cancel_requested:
                break

            # Check timeout
            elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
            if elapsed > MOVE_TIMEOUT:
                self.get_logger().warn('Linear move timed out.')
                break

            # How far have we traveled?
            dx = self.current_x - start_x
            dy = self.current_y - start_y
            traveled  = math.sqrt(dx * dx + dy * dy)
            remaining = target - traveled

            if remaining <= DISTANCE_TOLERANCE:
                break

            # Proportional slow-down near goal
            speed = linear_speed
            if remaining < DISTANCE_SLOWDOWN:
                speed = max(
                    MIN_LINEAR_SPEED,
                    linear_speed * (remaining / DISTANCE_SLOWDOWN)
                )

            # ── Heading hold ──────────────────────────────────────────────────
            # How far have we drifted from our starting heading?
            yaw_error = angle_diff(start_yaw, self.current_yaw)
            # Proportional correction — positive error means drifted right, correct left
            angular_correction = HEADING_KP * yaw_error
            # Cap the correction so it doesn't overpower forward motion
            angular_correction = max(-MAX_HEADING_CORRECTION,
                                     min(MAX_HEADING_CORRECTION, angular_correction))
            # ─────────────────────────────────────────────────────────────────

            # Publish cmd_vel with heading correction applied
            twist = Twist()
            twist.linear.x  = sign * speed
            twist.angular.z = angular_correction
            self.cmd_pub.publish(twist)

            # Feedback
            feedback.distance_remaining = remaining
            feedback.angle_remaining    = abs(0.0)
            feedback.current_speed      = speed
            goal_handle.publish_feedback(feedback)

            rate.sleep()

        traveled = math.sqrt(
            (self.current_x - start_x) ** 2 +
            (self.current_y - start_y) ** 2
        )
        return sign * traveled

    # ── Turn phase ────────────────────────────────────────────────────────────

    def _run_turn(self, goal_handle, feedback, target_degrees, angular_speed_rad):
        """Turn target_degrees. Positive=left (CCW), negative=right (CW).
        Returns degrees actually turned."""
        target_rad  = math.radians(target_degrees)
        start_yaw   = self.current_yaw
        goal_yaw    = start_yaw + target_rad
        sign        = 1.0 if target_degrees >= 0 else -1.0

        start_time = self.get_clock().now()
        rate = self.create_rate(20)

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                break

            elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
            if elapsed > MOVE_TIMEOUT:
                self.get_logger().warn('Turn timed out.')
                break

            remaining_rad = abs(angle_diff(goal_yaw, self.current_yaw))
            remaining_deg = math.degrees(remaining_rad)

            if remaining_deg <= ANGLE_TOLERANCE:
                break

            # Proportional slow-down near goal
            speed = angular_speed_rad
            if remaining_deg < ANGLE_SLOWDOWN:
                speed = max(
                    math.radians(MIN_ANGULAR_SPEED),
                    angular_speed_rad * (remaining_deg / ANGLE_SLOWDOWN)
                )

            twist = Twist()
            twist.angular.z = sign * speed
            self.cmd_pub.publish(twist)

            feedback.distance_remaining = 0.0
            feedback.angle_remaining    = remaining_deg
            feedback.current_speed      = 0.0
            goal_handle.publish_feedback(feedback)

            rate.sleep()

        turned_rad = angle_diff(self.current_yaw, start_yaw)
        return math.degrees(turned_rad)

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _stop(self):
        self.cmd_pub.publish(Twist())


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = MoveDistanceServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()