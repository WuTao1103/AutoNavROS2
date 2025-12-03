#!/usr/bin/env python3
"""
Path Planning Service - Cleaned version
Removed unnecessary cmd_vel publisher
"""

import rclpy
from rclpy.node import Node
import time
import math
import heapq
import numpy as np
import threading

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from path_planning.srv import PlanPath
from std_msgs.msg import Header


class Node2D:
    __slots__ = ['x', 'y', 'cost', 'heuristic', 'parent']
    
    def __init__(self, x, y, cost=0.0, heuristic=0.0, parent=None):
        self.x = x
        self.y = y
        self.cost = cost
        self.heuristic = heuristic
        self.parent = parent

    @property
    def f_cost(self):
        return self.cost + self.heuristic

    def __lt__(self, other):
        return self.f_cost < other.f_cost


class PathPlanningService(Node):
    def __init__(self):
        super().__init__('path_planning_service')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('service_name', 'plan_path'),
                ('path_topic', '/path'),
                ('map_topic', '/map'),
                ('publish_path', True),
                ('allow_diagonal', True),
                ('heuristic_weight', 1.0),
                ('inflation_radius', 0.4),
                ('obstacle_threshold', 50),
            ])

        self.service_name = self.get_parameter('service_name').value
        self.path_topic = self.get_parameter('path_topic').value
        self.publish_path_flag = self.get_parameter('publish_path').value
        self.allow_diagonal = self.get_parameter('allow_diagonal').value
        self.heuristic_weight = self.get_parameter('heuristic_weight').value
        self.inflation_radius = self.get_parameter('inflation_radius').value
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').value

        self.map_lock = threading.Lock()
        self.map_data = None
        self.map_info = None
        self.inflated_map = None

        # Services and publishers
        self.path_service = self.create_service(
            PlanPath, 
            self.service_name, 
            self.plan_path_callback
        )
        
        if self.publish_path_flag:
            self.path_pub = self.create_publisher(Path, self.path_topic, 10)
        
        # Map subscriber
        self.map_sub = self.create_subscription(
            OccupancyGrid, 
            self.get_parameter('map_topic').value, 
            self.map_callback, 
            10
        )

        self.get_logger().info("=" * 60)
        self.get_logger().info(f"Path Planning Service")
        self.get_logger().info(f"  Service: {self.service_name}")
        self.get_logger().info(f"  Path topic: {self.path_topic}")
        self.get_logger().info(f"  Inflation radius: {self.inflation_radius}m")
        self.get_logger().info(f"  Allow diagonal: {self.allow_diagonal}")
        self.get_logger().info("=" * 60)

    def map_callback(self, msg):
        with self.map_lock:
            self.map_data = np.array(msg.data, dtype=np.int8).reshape(
                msg.info.height, msg.info.width
            )
            self.map_info = msg.info
            
            inflation_cells = int(math.ceil(
                self.inflation_radius / msg.info.resolution
            ))
            self.inflated_map = self._inflate_obstacles(
                self.map_data, inflation_cells
            )
            
            orig = np.sum(self.map_data > self.obstacle_threshold)
            infl = np.sum(self.inflated_map > self.obstacle_threshold)
            self.get_logger().info(
                f"[MAP] {msg.info.width}x{msg.info.height}, "
                f"inflation={inflation_cells} cells, "
                f"obstacles: {orig}->{infl}"
            )

    def _inflate_obstacles(self, grid, inflation_cells):
        """Inflate obstacles"""
        binary = (grid > self.obstacle_threshold).astype(np.uint8)
        inflated = np.zeros_like(binary)
        height, width = binary.shape
        
        obstacle_y, obstacle_x = np.where(binary > 0)
        
        for oy, ox in zip(obstacle_y, obstacle_x):
            for dy in range(-inflation_cells, inflation_cells + 1):
                for dx in range(-inflation_cells, inflation_cells + 1):
                    if dx*dx + dy*dy <= inflation_cells*inflation_cells:
                        ny, nx = oy + dy, ox + dx
                        if 0 <= ny < height and 0 <= nx < width:
                            inflated[ny, nx] = 1
        
        result = grid.copy()
        result[inflated > 0] = 100
        return result

    def world_to_grid(self, wx, wy):
        """Convert world coordinates to grid coordinates"""
        if self.map_info is None:
            return None, None
        ox = self.map_info.origin.position.x
        oy = self.map_info.origin.position.y
        res = self.map_info.resolution
        gx = int(round((wx - ox) / res))
        gy = int(round((wy - oy) / res))
        return gx, gy

    def grid_to_world(self, gx, gy):
        """Convert grid coordinates to world coordinates"""
        if self.map_info is None:
            return None, None
        ox = self.map_info.origin.position.x
        oy = self.map_info.origin.position.y
        res = self.map_info.resolution
        wx = ox + (gx + 0.5) * res
        wy = oy + (gy + 0.5) * res
        return wx, wy

    def is_valid(self, gx, gy, inflated_map):
        """
        Check if grid cell is valid
        Note: numpy array indexing is [row, col] = [y, x]
        """
        height, width = inflated_map.shape
        
        # Boundary check
        if gx < 0 or gx >= width or gy < 0 or gy >= height:
            return False
        
        # Obstacle check - note indexing order is [y, x]
        cell_value = inflated_map[gy, gx]

        # Handle unknown areas (-1) as obstacles for safety
        if cell_value == -1:
            return False

        return cell_value < self.obstacle_threshold

    def get_neighbors(self, x, y, inflated_map):
        """Get neighbor nodes"""
        neighbors = []
        
        if self.allow_diagonal:
            directions = [
                (-1,-1), (-1,0), (-1,1),
                (0,-1),          (0,1),
                (1,-1),  (1,0),  (1,1)
            ]
        else:
            directions = [(-1,0), (1,0), (0,-1), (0,1)]
        
        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            
            if not self.is_valid(nx, ny, inflated_map):
                continue
            
            # Check corner cutting when moving diagonally
            if abs(dx) + abs(dy) == 2:
                if not self.is_valid(x + dx, y, inflated_map):
                    continue
                if not self.is_valid(x, y + dy, inflated_map):
                    continue
                cost = math.sqrt(2)
            else:
                cost = 1.0
            
            neighbors.append((nx, ny, cost))
        
        return neighbors

    def heuristic(self, x1, y1, x2, y2):
        """Heuristic function"""
        dx, dy = abs(x2 - x1), abs(y2 - y1)
        if self.allow_diagonal:
            # Diagonal distance (improved Chebyshev distance)
            return max(dx, dy) + (math.sqrt(2) - 1) * min(dx, dy)
        return dx + dy

    def plan_astar(self, start_world, goal_world):
        """A* path planning"""
        with self.map_lock:
            if self.inflated_map is None:
                self.get_logger().error("[PLAN] No map available")
                return None
            inflated_map = self.inflated_map.copy()
        
        # Convert to grid coordinates
        sx, sy = self.world_to_grid(start_world[0], start_world[1])
        gx, gy = self.world_to_grid(goal_world[0], goal_world[1])
        
        if sx is None or gx is None:
            self.get_logger().error("[PLAN] Invalid coordinates")
            return None
        
        height, width = inflated_map.shape
        self.get_logger().info(
            f"[PLAN] Grid: ({sx},{sy}) -> ({gx},{gy}), "
            f"Map: {width}x{height}"
        )
        
        # Check start and goal points
        if not self.is_valid(sx, sy, inflated_map):
            self.get_logger().error(
                f"[PLAN] Start ({sx},{sy}) in obstacle! "
                f"Value={inflated_map[sy, sx]}"
            )
            return None
        
        if not self.is_valid(gx, gy, inflated_map):
            self.get_logger().error(
                f"[PLAN] Goal ({gx},{gy}) in obstacle! "
                f"Value={inflated_map[gy, gx]}"
            )
            return None
        
        # A* search
        open_list = []
        closed_set = set()
        
        start_node = Node2D(
            sx, sy, 0.0, 
            self.heuristic_weight * self.heuristic(sx, sy, gx, gy)
        )
        heapq.heappush(open_list, start_node)
        all_nodes = {(sx, sy): start_node}
        
        iterations = 0
        max_iterations = 100000
        
        while open_list and iterations < max_iterations:
            iterations += 1
            
            current = heapq.heappop(open_list)
            cx, cy = current.x, current.y
            
            # Reached goal
            if cx == gx and cy == gy:
                self.get_logger().info(
                    f"[PLAN] Path found in {iterations} iterations"
                )
                
                # Reconstruct path
                path = []
                node = current
                while node:
                    wx, wy = self.grid_to_world(node.x, node.y)
                    path.append((wx, wy))
                    node = node.parent
                
                path.reverse()
                self.get_logger().info(f"[PLAN] Path has {len(path)} points")
                return path
            
            if (cx, cy) in closed_set:
                continue
            closed_set.add((cx, cy))
            
            # Expand neighbors
            for nx, ny, move_cost in self.get_neighbors(cx, cy, inflated_map):
                if (nx, ny) in closed_set:
                    continue
                
                new_cost = current.cost + move_cost
                
                if (nx, ny) in all_nodes:
                    existing = all_nodes[(nx, ny)]
                    if new_cost < existing.cost:
                        existing.cost = new_cost
                        existing.parent = current
                        heapq.heappush(open_list, existing)
                else:
                    h = self.heuristic_weight * self.heuristic(nx, ny, gx, gy)
                    new_node = Node2D(nx, ny, new_cost, h, current)
                    all_nodes[(nx, ny)] = new_node
                    heapq.heappush(open_list, new_node)
        
        self.get_logger().error(
            f"[PLAN] No path found after {iterations} iterations"
        )
        return None

    def plan_path_callback(self, request, response):
        """Path planning service callback"""
        t0 = time.time()
        
        try:
            start = (
                request.start.pose.position.x, 
                request.start.pose.position.y
            )
            goal = (
                request.goal.pose.position.x, 
                request.goal.pose.position.y
            )
            
            self.get_logger().info(
                f"Planning: ({start[0]:.2f},{start[1]:.2f}) -> "
                f"({goal[0]:.2f},{goal[1]:.2f})"
            )
            
            # Plan path
            path = self.plan_astar(start, goal)
            dt = time.time() - t0
            
            if not path:
                response.success = False
                response.message = "No path found"
                response.planning_time = dt
                return response
            
            # Create path message
            path_msg = Path()
            path_msg.header.frame_id = request.start.header.frame_id or "map"
            path_msg.header.stamp = self.get_clock().now().to_msg()
            
            for i, (x, y) in enumerate(path):
                pose = PoseStamped()
                pose.header = path_msg.header
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.position.z = 0.0
                
                # Calculate orientation
                if i < len(path) - 1:
                    nx, ny = path[i + 1]
                    yaw = math.atan2(ny - y, nx - x)
                    pose.pose.orientation.z = math.sin(yaw / 2.0)
                    pose.pose.orientation.w = math.cos(yaw / 2.0)
                else:
                    pose.pose.orientation.w = 1.0
                
                path_msg.poses.append(pose)
            
            # Publish path for visualization
            if self.publish_path_flag and self.path_pub:
                self.path_pub.publish(path_msg)
            
            response.success = True
            response.path = path_msg
            response.message = f"Path planned with {len(path)} points"
            response.planning_time = dt
            
            self.get_logger().info(
                f"✓ Planning succeeded: {len(path)} points, {dt:.3f}s"
            )
            
        except Exception as e:
            import traceback
            self.get_logger().error(f"Planning error: {e}")
            traceback.print_exc()
            response.success = False
            response.message = str(e)
            response.planning_time = time.time() - t0
        
        return response


def main(args=None):
    rclpy.init(args=args)
    node = PathPlanningService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()