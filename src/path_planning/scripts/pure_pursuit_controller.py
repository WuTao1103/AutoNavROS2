#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
import numpy as np

from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Path
from tf2_ros import TransformListener, Buffer
from tf2_geometry_msgs import do_transform_pose
import tf2_ros


class PurePursuitController(Node):
    def __init__(self):
        super().__init__('pure_pursuit_controller')

        # Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('lookahead_distance', 0.5),
                ('max_linear_velocity', 0.3),
                ('max_angular_velocity', 1.0),
                ('goal_tolerance', 0.1),
                ('cmd_vel_topic', '/cmd_vel'),
                ('path_topic', '/path'),
                ('robot_frame', 'base_link'),
                ('global_frame', 'map')
            ])

        # Get parameters
        self.lookahead_distance = self.get_parameter('lookahead_distance').value
        self.max_linear_vel = self.get_parameter('max_linear_velocity').value
        self.max_angular_vel = self.get_parameter('max_angular_velocity').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.global_frame = self.get_parameter('global_frame').value

        # Publishers and Subscribers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            self.get_parameter('cmd_vel_topic').value,
            10
        )

        self.path_sub = self.create_subscription(
            Path,
            self.get_parameter('path_topic').value,
            self.path_callback,
            10
        )

        # TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # State variables
        self.current_path = None
        self.current_pose = None
        self.goal_reached = False

        # Control timer
        self.control_timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("Pure Pursuit Controller initialized")

    def path_callback(self, msg):
        """Receive new path to follow"""
        self.current_path = msg
        self.goal_reached = False
        self.get_logger().info(f"Received new path with {len(msg.poses)} points")

    def get_robot_pose(self):
        """Get current robot pose in global frame"""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.robot_frame,
                rclpy.time.Time()
            )

            # Create pose from transform
            pose = PoseStamped()
            pose.header.frame_id = self.global_frame
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.orientation = transform.transform.rotation

            return pose

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f"Failed to get robot pose: {e}")
            return None

    def find_lookahead_point(self, path, robot_pose):
        """Find the lookahead point on the path"""
        if not path or not path.poses:
            return None

        robot_x = robot_pose.pose.position.x
        robot_y = robot_pose.pose.position.y

        # Find the closest point on path
        min_distance = float('inf')
        closest_index = 0

        for i, pose in enumerate(path.poses):
            px = pose.pose.position.x
            py = pose.pose.position.y
            distance = math.sqrt((px - robot_x)**2 + (py - robot_y)**2)

            if distance < min_distance:
                min_distance = distance
                closest_index = i

        # Look for lookahead point starting from closest point
        for i in range(closest_index, len(path.poses)):
            px = path.poses[i].pose.position.x
            py = path.poses[i].pose.position.y
            distance = math.sqrt((px - robot_x)**2 + (py - robot_y)**2)

            if distance >= self.lookahead_distance:
                return Point(x=px, y=py, z=0.0)

        # If no lookahead point found, return the last point
        last_pose = path.poses[-1]
        return Point(
            x=last_pose.pose.position.x,
            y=last_pose.pose.position.y,
            z=0.0
        )

    def calculate_steering_angle(self, robot_pose, target_point):
        """Calculate steering angle using pure pursuit geometry"""
        robot_x = robot_pose.pose.position.x
        robot_y = robot_pose.pose.position.y

        # Get robot heading
        orientation = robot_pose.pose.orientation
        robot_yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
        )

        # Calculate angle to target
        target_angle = math.atan2(
            target_point.y - robot_y,
            target_point.x - robot_x
        )

        # Calculate steering angle (difference between target angle and robot heading)
        steering_angle = target_angle - robot_yaw

        # Normalize angle to [-pi, pi]
        while steering_angle > math.pi:
            steering_angle -= 2 * math.pi
        while steering_angle < -math.pi:
            steering_angle += 2 * math.pi

        return steering_angle

    def is_goal_reached(self, robot_pose, path):
        """Check if robot has reached the goal"""
        if not path or not path.poses:
            return True

        goal_pose = path.poses[-1]
        robot_x = robot_pose.pose.position.x
        robot_y = robot_pose.pose.position.y
        goal_x = goal_pose.pose.position.x
        goal_y = goal_pose.pose.position.y

        distance = math.sqrt((goal_x - robot_x)**2 + (goal_y - robot_y)**2)
        return distance < self.goal_tolerance

    def control_loop(self):
        """Main control loop"""
        if self.goal_reached or not self.current_path:
            # Stop the robot
            cmd_vel = Twist()
            self.cmd_vel_pub.publish(cmd_vel)
            return

        # Get current robot pose
        robot_pose = self.get_robot_pose()
        if not robot_pose:
            return

        # Check if goal is reached
        if self.is_goal_reached(robot_pose, self.current_path):
            self.goal_reached = True
            self.get_logger().info("Goal reached!")
            cmd_vel = Twist()
            self.cmd_vel_pub.publish(cmd_vel)
            return

        # Find lookahead point
        target_point = self.find_lookahead_point(self.current_path, robot_pose)
        if not target_point:
            return

        # Calculate steering angle
        steering_angle = self.calculate_steering_angle(robot_pose, target_point)

        # Create velocity command
        cmd_vel = Twist()

        # Linear velocity (reduce when turning)
        linear_vel = self.max_linear_vel * (1.0 - abs(steering_angle) / math.pi)
        cmd_vel.linear.x = max(0.1, linear_vel)  # Minimum forward speed

        # Angular velocity
        cmd_vel.angular.z = max(-self.max_angular_vel,
                               min(self.max_angular_vel, steering_angle * 2.0))

        # Publish command
        self.cmd_vel_pub.publish(cmd_vel)

        # Debug info
        self.get_logger().debug(
            f"Target: ({target_point.x:.2f}, {target_point.y:.2f}), "
            f"Steering: {math.degrees(steering_angle):.1f}°, "
            f"Linear: {cmd_vel.linear.x:.2f}, Angular: {cmd_vel.angular.z:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)

    controller = PurePursuitController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()