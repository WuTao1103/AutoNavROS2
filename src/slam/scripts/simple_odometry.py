#!/usr/bin/env python3
"""
Simple Odometry Node - Dead Reckoning for TurtleBot
Estimates robot pose based on velocity commands and motion model
"""

import rclpy
from rclpy.node import Node
import math
import time
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Header


class SimpleOdometry(Node):
    def __init__(self):
        super().__init__('simple_odometry')

        # Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('cmd_vel_topic', '/cmd_vel'),
                ('odom_topic', '/odom'),
                ('odom_frame', 'odom'),
                ('base_frame', 'base_link'),
                ('publish_rate', 50.0),        # Hz
                ('wheel_base', 0.16),          # meters - TurtleBot wheelbase
                ('wheel_radius', 0.033),       # meters - TurtleBot wheel radius
                ('use_cmd_vel_estimation', True), # Use cmd_vel for estimation
            ])

        # Get parameters
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.wheel_base = self.get_parameter('wheel_base').get_parameter_value().double_value
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.use_cmd_vel = self.get_parameter('use_cmd_vel_estimation').get_parameter_value().bool_value

        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vtheta = 0.0

        # Timing
        self.last_time = time.time()
        self.current_cmd_vel = None

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Publishers and subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            self.get_parameter('cmd_vel_topic').get_parameter_value().string_value,
            self.cmd_vel_callback,
            10)

        self.odom_pub = self.create_publisher(
            Odometry,
            self.get_parameter('odom_topic').get_parameter_value().string_value,
            10)

        # Timer for publishing odometry
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.timer = self.create_timer(1.0 / publish_rate, self.update_odometry)

        self.get_logger().info('Simple Odometry node initialized')
        self.get_logger().info(f'Publishing odometry at {publish_rate} Hz')
        self.get_logger().info(f'Odometry frame: {self.odom_frame}')
        self.get_logger().info(f'Base frame: {self.base_frame}')

    def cmd_vel_callback(self, msg):
        """Store the latest velocity command"""
        self.current_cmd_vel = msg

    def update_odometry(self):
        """Update robot pose based on motion model and publish odometry"""
        current_time = time.time()
        dt = current_time - self.last_time

        if dt <= 0.0:
            return

        # Use velocity commands for estimation if available
        if self.use_cmd_vel and self.current_cmd_vel is not None:
            linear_vel = self.current_cmd_vel.linear.x
            angular_vel = self.current_cmd_vel.angular.z

            # Update velocities
            self.vx = linear_vel * math.cos(self.theta)
            self.vy = linear_vel * math.sin(self.theta)
            self.vtheta = angular_vel

            # Integrate to get position
            # Use simple Euler integration
            if abs(angular_vel) < 1e-6:  # Straight line motion
                self.x += linear_vel * math.cos(self.theta) * dt
                self.y += linear_vel * math.sin(self.theta) * dt
            else:  # Curved motion
                # More accurate integration for curved motion
                dtheta = angular_vel * dt
                r = linear_vel / angular_vel  # Radius of curvature

                # Calculate the center of the circular arc
                cx = self.x - r * math.sin(self.theta)
                cy = self.y + r * math.cos(self.theta)

                # Update theta first
                self.theta += dtheta

                # Calculate new position
                self.x = cx + r * math.sin(self.theta)
                self.y = cy - r * math.cos(self.theta)

        # Normalize theta to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Create and publish odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame

        # Position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        # Orientation (quaternion from yaw)
        quat = self.yaw_to_quaternion(self.theta)
        odom_msg.pose.pose.orientation = quat

        # Velocity
        odom_msg.twist.twist.linear.x = self.vx
        odom_msg.twist.twist.linear.y = self.vy
        odom_msg.twist.twist.angular.z = self.vtheta

        # Covariance (simple diagonal matrix)
        # Position covariance
        odom_msg.pose.covariance[0] = 0.1   # x
        odom_msg.pose.covariance[7] = 0.1   # y
        odom_msg.pose.covariance[35] = 0.2  # theta

        # Velocity covariance
        odom_msg.twist.covariance[0] = 0.1   # vx
        odom_msg.twist.covariance[7] = 0.1   # vy
        odom_msg.twist.covariance[35] = 0.2  # vtheta

        # Publish odometry
        self.odom_pub.publish(odom_msg)

        # Publish TF transform
        self.publish_tf_transform(odom_msg.header.stamp, quat)

        self.last_time = current_time

    def yaw_to_quaternion(self, yaw):
        """Convert yaw angle to quaternion"""
        quat = Quaternion()
        quat.x = 0.0
        quat.y = 0.0
        quat.z = math.sin(yaw / 2.0)
        quat.w = math.cos(yaw / 2.0)
        return quat

    def publish_tf_transform(self, timestamp, quaternion):
        """Publish TF transform from odom to base_link"""
        transform = TransformStamped()
        transform.header.stamp = timestamp
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame

        # Translation
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0

        # Rotation
        transform.transform.rotation = quaternion

        self.tf_broadcaster.sendTransform(transform)

    def reset_odometry(self):
        """Reset odometry to origin"""
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vtheta = 0.0
        self.get_logger().info('Odometry reset to origin')


def main(args=None):
    rclpy.init(args=args)
    node = SimpleOdometry()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()