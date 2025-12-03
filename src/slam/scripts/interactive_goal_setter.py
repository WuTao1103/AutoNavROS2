#!/usr/bin/env python3
"""
Interactive Goal Setter - Command-line interactive goal point setting tool
Allows users to input goal coordinates via command line, automatically calls path planning service
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from path_planning.srv import PlanPath
import math


class InteractiveGoalSetter(Node):
    def __init__(self):
        super().__init__('interactive_goal_setter')

        # Create service client
        self.client = self.create_client(PlanPath, 'plan_path')

        # Wait for service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for path planning service...')

        self.get_logger().info('✅ Path planning service connected')

    def send_goal(self, x, y, yaw_degrees=0.0):
        """Send goal point to path planning service"""
        request = PlanPath.Request()

        # Start point (assumed at origin)
        request.start = PoseStamped()
        request.start.header.frame_id = "map"
        request.start.header.stamp = self.get_clock().now().to_msg()
        request.start.pose.position = Point(x=0.0, y=0.0, z=0.0)
        request.start.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        # Goal point
        request.goal = PoseStamped()
        request.goal.header.frame_id = "map"
        request.goal.header.stamp = self.get_clock().now().to_msg()
        request.goal.pose.position = Point(x=x, y=y, z=0.0)

        # Convert angle to quaternion
        yaw_rad = math.radians(yaw_degrees)
        request.goal.pose.orientation = Quaternion(
            x=0.0, y=0.0,
            z=math.sin(yaw_rad/2.0),
            w=math.cos(yaw_rad/2.0)
        )

        # Call service
        self.get_logger().info(f'🎯 Sending goal point: ({x:.2f}, {y:.2f}, {yaw_degrees}°)')
        future = self.client.call_async(request)
        return future


def main(args=None):
    rclpy.init(args=args)
    node = InteractiveGoalSetter()

    print("=" * 50)
    print("  🎮 Interactive Goal Point Setting Tool")
    print("=" * 50)
    print("")
    print("Usage:")
    print("  Enter goal coordinates (x, y, angle)")
    print("  Example: 2.0 3.0 90")
    print("  Enter 'q' or 'quit' to exit")
    print("")
    print("Coordinate system:")
    print("  📍 Map origin: (0, 0)")
    print("  📐 Angle: 0° = East, 90° = North, 180° = West, 270° = South")
    print("")

    try:
        while True:
            try:
                user_input = input("🎯 Please enter goal (x y angle): ").strip()

                if user_input.lower() in ['q', 'quit', 'exit']:
                    print("👋 Goodbye!")
                    break

                if not user_input:
                    continue

                # Parse input
                parts = user_input.split()
                if len(parts) >= 2:
                    x = float(parts[0])
                    y = float(parts[1])
                    yaw = float(parts[2]) if len(parts) > 2 else 0.0

                    # Send goal
                    future = node.send_goal(x, y, yaw)
                    rclpy.spin_until_future_complete(node, future)

                    if future.result() is not None:
                        result = future.result()
                        if result.success:
                            print(f"✅ Path planning successful! Planning time: {result.planning_time:.3f}s")
                            print(f"   Path contains {len(result.path.poses)} waypoints")
                        else:
                            print(f"❌ Path planning failed: {result.message}")
                    else:
                        print("❌ Service call failed")
                else:
                    print("❌ Invalid input format, please enter: x y [angle]")

            except ValueError:
                print("❌ Please enter valid numbers")
            except KeyboardInterrupt:
                print("\n👋 Goodbye!")
                break

    except Exception as e:
        node.get_logger().error(f"Program error: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()