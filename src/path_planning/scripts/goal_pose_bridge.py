#!/usr/bin/env python3
"""
Goal Pose Bridge - Connect RViz 2D Nav Goal with path planning service
Listen to /goal_pose topic, automatically call path planning service and publish path
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from nav_msgs.msg import Path, Odometry
from path_planning.srv import PlanPath
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose_stamped
import tf2_ros
import math


class GoalPoseBridge(Node):
    def __init__(self):
        super().__init__('goal_pose_bridge')

        # Subscribe to goal point published by RViz
        self.goal_subscription = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_pose_callback,
            10
        )

        # Subscribe to robot odometry
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Create path planning service client
        self.planning_client = self.create_client(PlanPath, '/plan_path')

        # Publish planned path
        self.path_publisher = self.create_publisher(Path, '/path', 10)

        # TF buffer and listener for coordinate transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Wait for service to be available
        self.get_logger().info('Goal Pose Bridge starting...')
        self.get_logger().info('Waiting for path planning service...')

        while not self.planning_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for path planning service /plan_path ...')

        self.get_logger().info('Goal Pose Bridge ready!')
        self.get_logger().info('Please use 2D Nav Goal tool in RViz to set target')

        # Robot pose tracking
        self.current_robot_pose = None
        self.latest_odom = None
        self.setup_default_robot_pose()

    def setup_default_robot_pose(self):
        """Set default robot position (will be updated from odometry)"""
        self.current_robot_pose = PoseStamped()
        self.current_robot_pose.header.frame_id = "map"
        self.current_robot_pose.pose.position = Point(x=0.0, y=0.0, z=0.0)
        self.current_robot_pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

    def odom_callback(self, msg):
        """Update robot position from odometry"""
        self.latest_odom = msg
        self.update_robot_pose_in_map()

    def update_robot_pose_in_map(self):
        """Transform robot pose from odom frame to map frame"""
        if self.latest_odom is None:
            return

        try:
            # Create pose in odom frame
            odom_pose = PoseStamped()
            odom_pose.header = self.latest_odom.header
            odom_pose.pose = self.latest_odom.pose.pose

            # Transform to map frame
            transform = self.tf_buffer.lookup_transform(
                'map', 'odom', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))

            # Use do_transform_pose_stamped for PoseStamped objects
            map_pose = do_transform_pose_stamped(odom_pose, transform)
            self.current_robot_pose = map_pose

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            # If TF lookup fails, use odom pose directly (assuming map = odom)
            self.current_robot_pose = PoseStamped()
            self.current_robot_pose.header.frame_id = "map"
            self.current_robot_pose.header.stamp = self.latest_odom.header.stamp
            self.current_robot_pose.pose = self.latest_odom.pose.pose

            if not hasattr(self, '_tf_warning_logged'):
                self._tf_warning_logged = False
            if not self._tf_warning_logged:
                self.get_logger().warn(f"TF lookup failed, using odom pose directly: {e}")
                self._tf_warning_logged = True

    def goal_pose_callback(self, msg):
        """Handle goal point received from RViz"""
        self.get_logger().info(f'Received new goal: '
                              f'({msg.pose.position.x:.2f}, '
                              f'{msg.pose.position.y:.2f})')

        # Update robot pose before planning
        if self.latest_odom is not None:
            self.update_robot_pose_in_map()
            self.get_logger().info(f'Current robot position: '
                                  f'({self.current_robot_pose.pose.position.x:.2f}, '
                                  f'{self.current_robot_pose.pose.position.y:.2f})')

        # Call path planning service
        self.call_planning_service(msg)

    def call_planning_service(self, goal_pose):
        """Call path planning service"""
        request = PlanPath.Request()

        # Set start point (current robot position)
        request.start = self.current_robot_pose
        request.start.header.stamp = self.get_clock().now().to_msg()

        # Set goal point
        request.goal = goal_pose
        request.goal.header.stamp = self.get_clock().now().to_msg()

        # Call service asynchronously
        self.get_logger().info('🚀 Planning path...')
        future = self.planning_client.call_async(request)
        future.add_done_callback(self.planning_response_callback)

    def planning_response_callback(self, future):
        """Handle path planning service response"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'✅ Path planning successful!')
                self.get_logger().info(f'   Planning time: {response.planning_time:.3f}s')
                self.get_logger().info(f'   Path points: {len(response.path.poses)}')

                # Publish path to /path topic (for Pure Pursuit controller)
                self.path_publisher.publish(response.path)
                self.get_logger().info('📤 Path published to /path topic')
            else:
                self.get_logger().error(f'❌ Path planning failed: {response.message}')
        except Exception as e:
            self.get_logger().error(f'❌ Service call exception: {str(e)}')


def main(args=None):
    rclpy.init(args=args)

    try:
        node = GoalPoseBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        import traceback
        print(f"Program exception: {e}")
        print(f"Traceback: {traceback.format_exc()}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()