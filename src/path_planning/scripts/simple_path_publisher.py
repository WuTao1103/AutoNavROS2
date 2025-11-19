#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header


class SimplePathPublisher(Node):
    def __init__(self):
        super().__init__('simple_path_publisher')

        # Publisher
        self.path_pub = self.create_publisher(Path, '/path', 10)

        # Timer to publish test paths
        self.timer = self.create_timer(5.0, self.publish_test_path)

        self.path_counter = 0
        self.get_logger().info("Simple Path Publisher initialized")

    def create_square_path(self, size=2.0, center_x=0.0, center_y=0.0):
        """Create a square path for testing"""
        path = Path()
        path.header = Header()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()

        # Square corners
        corners = [
            (center_x - size/2, center_y - size/2),  # Bottom left
            (center_x + size/2, center_y - size/2),  # Bottom right
            (center_x + size/2, center_y + size/2),  # Top right
            (center_x - size/2, center_y + size/2),  # Top left
            (center_x - size/2, center_y - size/2),  # Back to start
        ]

        for x, y in corners:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0

            # Simple orientation (facing forward)
            pose.pose.orientation.w = 1.0

            path.poses.append(pose)

        return path

    def create_circle_path(self, radius=1.5, center_x=0.0, center_y=0.0, points=16):
        """Create a circular path for testing"""
        path = Path()
        path.header = Header()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()

        for i in range(points + 1):  # +1 to close the circle
            angle = 2 * math.pi * i / points
            x = center_x + radius * math.cos(angle)
            y = center_y + radius * math.sin(angle)

            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0

            # Orient tangent to circle
            pose.pose.orientation.z = math.sin((angle + math.pi/2) / 2)
            pose.pose.orientation.w = math.cos((angle + math.pi/2) / 2)

            path.poses.append(pose)

        return path

    def create_figure_eight_path(self, radius=1.0, points=32):
        """Create a figure-eight path for testing"""
        path = Path()
        path.header = Header()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()

        for i in range(points):
            t = 2 * math.pi * i / points

            # Parametric equations for figure-eight
            x = radius * math.sin(t)
            y = radius * math.sin(t) * math.cos(t)

            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0

            # Calculate tangent direction
            dx = radius * math.cos(t)
            dy = radius * (math.cos(t)**2 - math.sin(t)**2)
            yaw = math.atan2(dy, dx)

            pose.pose.orientation.z = math.sin(yaw / 2)
            pose.pose.orientation.w = math.cos(yaw / 2)

            path.poses.append(pose)

        return path

    def create_straight_line_path(self, start_x=0.0, start_y=0.0, end_x=3.0, end_y=0.0, points=10):
        """Create a straight line path for basic testing"""
        path = Path()
        path.header = Header()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()

        for i in range(points):
            t = i / (points - 1)  # 0 to 1
            x = start_x + t * (end_x - start_x)
            y = start_y + t * (end_y - start_y)

            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0

            # Face forward along the line
            yaw = math.atan2(end_y - start_y, end_x - start_x)
            pose.pose.orientation.z = math.sin(yaw / 2)
            pose.pose.orientation.w = math.cos(yaw / 2)

            path.poses.append(pose)

        return path

    def publish_test_path(self):
        """Publish different test paths cyclically"""
        paths = {
            0: ("straight line", self.create_straight_line_path()),
            1: ("square", self.create_square_path()),
            2: ("circle", self.create_circle_path()),
            3: ("figure eight", self.create_figure_eight_path()),
        }

        path_type = self.path_counter % len(paths)
        path_name, path = paths[path_type]

        self.path_pub.publish(path)
        self.get_logger().info(f"Published {path_name} path with {len(path.poses)} points")

        self.path_counter += 1


def main(args=None):
    rclpy.init(args=args)

    publisher = SimplePathPublisher()

    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        pass
    finally:
        publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()