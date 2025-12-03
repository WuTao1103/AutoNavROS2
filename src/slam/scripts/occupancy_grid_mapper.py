#!/usr/bin/env python3
"""
Occupancy Grid Mapper - Simple SLAM implementation for TurtleBot
Uses LiDAR data to build 2D occupancy grid map in real-time
"""

import rclpy
from rclpy.node import Node
import numpy as np
import math
from collections import defaultdict

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Header
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs


class OccupancyGridMapper(Node):
    def __init__(self):
        super().__init__('occupancy_grid_mapper')

        # Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('map_width', 200),           # cells
                ('map_height', 200),          # cells
                ('map_resolution', 0.1),      # meters per cell
                ('map_origin_x', -10.0),      # meters
                ('map_origin_y', -10.0),      # meters
                ('scan_topic', '/scan'),
                ('odom_topic', '/odom'),
                ('map_topic', '/map'),
                ('map_frame', 'map'),
                ('robot_frame', 'base_link'),
                ('odom_frame', 'odom'),
                ('publish_rate', 2.0),        # Hz
                ('hit_probability', 0.7),     # Probability for occupied cell
                ('miss_probability', 0.3),    # Probability for free cell
                ('occupied_threshold', 0.65), # Threshold for occupied
                ('free_threshold', 0.35),     # Threshold for free
                ('max_range', 10.0),          # Maximum laser range to use
                ('min_range', 0.1),           # Minimum laser range to use
            ])

        # Get parameters
        self.map_width = self.get_parameter('map_width').get_parameter_value().integer_value
        self.map_height = self.get_parameter('map_height').get_parameter_value().integer_value
        self.map_resolution = self.get_parameter('map_resolution').get_parameter_value().double_value
        self.map_origin_x = self.get_parameter('map_origin_x').get_parameter_value().double_value
        self.map_origin_y = self.get_parameter('map_origin_y').get_parameter_value().double_value
        self.hit_prob = self.get_parameter('hit_probability').get_parameter_value().double_value
        self.miss_prob = self.get_parameter('miss_probability').get_parameter_value().double_value
        self.occupied_thresh = self.get_parameter('occupied_threshold').get_parameter_value().double_value
        self.free_thresh = self.get_parameter('free_threshold').get_parameter_value().double_value
        self.max_range = self.get_parameter('max_range').get_parameter_value().double_value
        self.min_range = self.get_parameter('min_range').get_parameter_value().double_value

        # Initialize occupancy grid - use log odds
        self.log_odds_map = np.zeros((self.map_height, self.map_width))
        self.log_hit = math.log(self.hit_prob / (1 - self.hit_prob))
        self.log_miss = math.log(self.miss_prob / (1 - self.miss_prob))

        # Current robot pose in map frame
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        # TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publishers and subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.get_parameter('scan_topic').get_parameter_value().string_value,
            self.scan_callback,
            10)

        self.odom_sub = self.create_subscription(
            Odometry,
            self.get_parameter('odom_topic').get_parameter_value().string_value,
            self.odom_callback,
            10)

        self.map_pub = self.create_publisher(
            OccupancyGrid,
            self.get_parameter('map_topic').get_parameter_value().string_value,
            1)

        # Timer for publishing map
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.map_timer = self.create_timer(1.0 / publish_rate, self.publish_map)

        self.get_logger().info('Occupancy Grid Mapper initialized')
        self.get_logger().info(f'Map size: {self.map_width}x{self.map_height} cells')
        self.get_logger().info(f'Resolution: {self.map_resolution} m/cell')
        self.get_logger().info(f'Origin: ({self.map_origin_x}, {self.map_origin_y})')

    def odom_callback(self, msg):
        """Update robot pose from odometry"""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        # Extract yaw from quaternion
        quat = msg.pose.pose.orientation
        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def scan_callback(self, msg):
        """Process laser scan and update occupancy grid"""
        if not hasattr(self, 'robot_x'):
            return  # Wait for odometry

        # Convert robot position to map coordinates
        robot_map_x, robot_map_y = self.world_to_map(self.robot_x, self.robot_y)

        if not self.is_valid_cell(robot_map_x, robot_map_y):
            return

        # Process each laser beam
        angle = msg.angle_min
        for i, range_val in enumerate(msg.ranges):
            if math.isnan(range_val) or math.isinf(range_val):
                angle += msg.angle_increment
                continue

            # Apply range limits
            if range_val < self.min_range or range_val > self.max_range:
                angle += msg.angle_increment
                continue

            # Calculate beam endpoint in world coordinates
            beam_angle = self.robot_yaw + angle
            end_x = self.robot_x + range_val * math.cos(beam_angle)
            end_y = self.robot_y + range_val * math.sin(beam_angle)

            # Convert to map coordinates
            end_map_x, end_map_y = self.world_to_map(end_x, end_y)

            # Trace beam and update cells
            self.trace_beam(robot_map_x, robot_map_y, end_map_x, end_map_y)

            angle += msg.angle_increment

    def world_to_map(self, world_x, world_y):
        """Convert world coordinates to map cell indices"""
        map_x = int((world_x - self.map_origin_x) / self.map_resolution)
        map_y = int((world_y - self.map_origin_y) / self.map_resolution)
        return map_x, map_y

    def is_valid_cell(self, map_x, map_y):
        """Check if cell coordinates are within map bounds"""
        return 0 <= map_x < self.map_width and 0 <= map_y < self.map_height

    def trace_beam(self, x0, y0, x1, y1):
        """Use Bresenham's line algorithm to trace beam and update cells"""
        cells = self.bresenham_line(x0, y0, x1, y1)

        # Mark all cells except the last one as free (ray passed through)
        for i, (x, y) in enumerate(cells[:-1]):
            if self.is_valid_cell(x, y):
                self.log_odds_map[y, x] += self.log_miss

        # Mark the endpoint as occupied (obstacle detected)
        if len(cells) > 0:
            x, y = cells[-1]
            if self.is_valid_cell(x, y):
                self.log_odds_map[y, x] += self.log_hit

    def bresenham_line(self, x0, y0, x1, y1):
        """Bresenham's line algorithm for ray tracing"""
        cells = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        x, y = x0, y0
        while True:
            cells.append((x, y))

            if x == x1 and y == y1:
                break

            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy

        return cells

    def log_odds_to_probability(self, log_odds):
        """Convert log odds to probability with overflow protection"""
        # Clamp log_odds to prevent overflow
        log_odds = max(-500.0, min(500.0, log_odds))

        try:
            exp_val = math.exp(log_odds)
            return 1.0 - 1.0 / (1.0 + exp_val)
        except OverflowError:
            # Handle overflow cases
            if log_odds > 0:
                return 1.0  # Very high confidence of occupied
            else:
                return 0.0  # Very high confidence of free

    def publish_map(self):
        """Publish the current occupancy grid map"""
        msg = OccupancyGrid()

        # Header
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter('map_frame').get_parameter_value().string_value

        # Map metadata
        msg.info.width = self.map_width
        msg.info.height = self.map_height
        msg.info.resolution = self.map_resolution
        msg.info.origin.position.x = self.map_origin_x
        msg.info.origin.position.y = self.map_origin_y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0

        # Convert log odds to occupancy probabilities
        occupancy_grid = np.zeros((self.map_height, self.map_width), dtype=np.int8)

        for y in range(self.map_height):
            for x in range(self.map_width):
                log_odds = self.log_odds_map[y, x]

                if abs(log_odds) < 1e-6:  # Unknown
                    occupancy_grid[y, x] = -1
                else:
                    prob = self.log_odds_to_probability(log_odds)
                    if prob > self.occupied_thresh:
                        occupancy_grid[y, x] = 100  # Occupied
                    elif prob < self.free_thresh:
                        occupancy_grid[y, x] = 0    # Free
                    else:
                        occupancy_grid[y, x] = -1   # Unknown

        # Flatten and assign to message
        msg.data = occupancy_grid.flatten().tolist()

        self.map_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OccupancyGridMapper()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()