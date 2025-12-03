#!/usr/bin/env python3
"""
A* Path Planner - Debug Version
Add detailed debug output to help locate path crossing obstacle issues
"""

import rclpy
from rclpy.node import Node
import math
import heapq
import numpy as np

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Header


class Node2D:
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


class AStarPlanner(Node):
    def __init__(self):
        super().__init__('astar_planner')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('map_topic', '/map'),
                ('allow_diagonal', True),
                ('heuristic_weight', 1.0),
                ('inflation_radius', 0.4),    # increased to 0.4m
                ('robot_radius', 0.2),
                ('enable_smoothing', False),  # disabled by default
                ('obstacle_threshold', 50),
                ('debug_mode', True),         # debug mode
            ])

        self.allow_diagonal = self.get_parameter('allow_diagonal').value
        self.heuristic_weight = self.get_parameter('heuristic_weight').value
        self.inflation_radius = self.get_parameter('inflation_radius').value
        self.robot_radius = self.get_parameter('robot_radius').value
        self.enable_smoothing = self.get_parameter('enable_smoothing').value
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').value
        self.debug_mode = self.get_parameter('debug_mode').value

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.get_parameter('map_topic').value,
            self.map_callback,
            10
        )

        self.map_data = None
        self.map_info = None
        self.inflated_map = None

        self.get_logger().info("=" * 60)
        self.get_logger().info("A* Planner DEBUG VERSION initialized")
        self.get_logger().info(f"  allow_diagonal: {self.allow_diagonal}")
        self.get_logger().info(f"  inflation_radius: {self.inflation_radius}m")
        self.get_logger().info(f"  enable_smoothing: {self.enable_smoothing}")
        self.get_logger().info(f"  debug_mode: {self.debug_mode}")
        self.get_logger().info("=" * 60)

    def map_callback(self, msg):
        """Receive and process occupancy grid map"""
        self.map_data = np.array(msg.data).reshape(msg.info.height, msg.info.width)
        self.map_info = msg.info
        self.inflated_map = self.inflate_obstacles(self.map_data)
        
        original_obstacles = np.sum(self.map_data > self.obstacle_threshold)
        inflated_obstacles = np.sum(self.inflated_map > self.obstacle_threshold)
        
        self.get_logger().info(
            f"[MAP] {msg.info.width}x{msg.info.height}, "
            f"resolution: {msg.info.resolution}m, "
            f"obstacles: {original_obstacles} -> {inflated_obstacles}"
        )

    def inflate_obstacles(self, grid_map):
        """Inflate obstacles"""
        if self.map_info is None:
            return grid_map

        inflation_cells = int(math.ceil(self.inflation_radius / self.map_info.resolution))
        
        self.get_logger().info(f"[INFLATE] radius={self.inflation_radius}m = {inflation_cells} cells")
        
        height, width = grid_map.shape
        inflated = grid_map.copy()

        obstacles = np.where(grid_map > self.obstacle_threshold)
        
        for oy, ox in zip(obstacles[0], obstacles[1]):
            for dy in range(-inflation_cells, inflation_cells + 1):
                for dx in range(-inflation_cells, inflation_cells + 1):
                    if dx * dx + dy * dy <= inflation_cells * inflation_cells:
                        ny, nx = oy + dy, ox + dx
                        if 0 <= ny < height and 0 <= nx < width:
                            inflated[ny, nx] = 100

        return inflated

    def world_to_grid(self, world_x, world_y):
        """Convert world coordinates to grid coordinates"""
        if self.map_info is None:
            return None, None

        origin_x = self.map_info.origin.position.x
        origin_y = self.map_info.origin.position.y
        resolution = self.map_info.resolution

        grid_x = int(round((world_x - origin_x) / resolution))
        grid_y = int(round((world_y - origin_y) / resolution))

        return grid_x, grid_y

    def grid_to_world(self, grid_x, grid_y):
        """Convert grid coordinates to world coordinates"""
        if self.map_info is None:
            return None, None

        origin_x = self.map_info.origin.position.x
        origin_y = self.map_info.origin.position.y
        resolution = self.map_info.resolution

        world_x = origin_x + (grid_x + 0.5) * resolution
        world_y = origin_y + (grid_y + 0.5) * resolution

        return world_x, world_y

    def is_valid_cell(self, x, y):
        """Check if grid cell is valid and not occupied"""
        if self.inflated_map is None:
            return False

        height, width = self.inflated_map.shape

        if x < 0 or x >= width or y < 0 or y >= height:
            return False

        return self.inflated_map[y, x] < self.obstacle_threshold

    def is_original_obstacle(self, x, y):
        """Check if it's an obstacle in the original map (not inflated)"""
        if self.map_data is None:
            return True
            
        height, width = self.map_data.shape
        if x < 0 or x >= width or y < 0 or y >= height:
            return True
            
        return self.map_data[y, x] > self.obstacle_threshold

    def heuristic_distance(self, x1, y1, x2, y2):
        """Calculate heuristic distance"""
        if self.allow_diagonal:
            dx = abs(x2 - x1)
            dy = abs(y2 - y1)
            return max(dx, dy) + (math.sqrt(2) - 1) * min(dx, dy)
        else:
            return abs(x2 - x1) + abs(y2 - y1)

    def get_neighbors(self, x, y):
        """
        Get valid neighboring cells
        Key fix: check corner cutting when moving diagonally
        """
        neighbors = []

        if self.allow_diagonal:
            directions = [
                (-1, -1), (-1, 0), (-1, 1),
                (0, -1),          (0, 1),
                (1, -1),  (1, 0),  (1, 1)
            ]
        else:
            directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]

        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            
            if not self.is_valid_cell(nx, ny):
                continue
            
            # ========== Key fix: diagonal corner cutting check ==========
            if abs(dx) + abs(dy) == 2:  # diagonal movement
                # Check if both adjacent straight directions are passable
                cell1_valid = self.is_valid_cell(x + dx, y)  # horizontal adjacent
                cell2_valid = self.is_valid_cell(x, y + dy)  # vertical adjacent
                
                if not cell1_valid or not cell2_valid:
                    if self.debug_mode:
                        self.get_logger().debug(
                            f"[CORNER CUT BLOCKED] ({x},{y})->({nx},{ny}): "
                            f"adjacent cells ({x+dx},{y})={cell1_valid}, ({x},{y+dy})={cell2_valid}"
                        )
                    continue  # prohibit corner cutting
                cost = math.sqrt(2)
            else:
                cost = 1.0
            # ================================================
                
            neighbors.append((nx, ny, cost))

        return neighbors

    def reconstruct_path(self, goal_node):
        """Reconstruct path from goal node"""
        path_points = []
        current = goal_node

        while current is not None:
            world_x, world_y = self.grid_to_world(current.x, current.y)
            if world_x is not None and world_y is not None:
                path_points.append((world_x, world_y))
            current = current.parent

        path_points.reverse()
        return path_points

    def validate_path(self, path_points):
        """
        Validate if path crosses obstacles
        This is a debug function to detect issues
        """
        if not self.debug_mode or self.map_data is None:
            return True, []
            
        violations = []
        
        for i, (wx, wy) in enumerate(path_points):
            gx, gy = self.world_to_grid(wx, wy)
            if gx is None or gy is None:
                continue
                
            # Check original map
            if self.is_original_obstacle(gx, gy):
                violations.append({
                    'index': i,
                    'world': (wx, wy),
                    'grid': (gx, gy),
                    'type': 'original_obstacle',
                    'value': self.map_data[gy, gx] if 0 <= gy < self.map_data.shape[0] and 0 <= gx < self.map_data.shape[1] else -1
                })
            # Check inflated map
            elif not self.is_valid_cell(gx, gy):
                violations.append({
                    'index': i,
                    'world': (wx, wy),
                    'grid': (gx, gy),
                    'type': 'inflated_obstacle',
                    'value': self.inflated_map[gy, gx] if 0 <= gy < self.inflated_map.shape[0] and 0 <= gx < self.inflated_map.shape[1] else -1
                })
        
        return len(violations) == 0, violations

    def plan_path(self, start_world, goal_world):
        """Plan path using A* algorithm"""
        if self.inflated_map is None:
            self.get_logger().error("[PLAN] No map available")
            return None

        start_x, start_y = self.world_to_grid(start_world[0], start_world[1])
        goal_x, goal_y = self.world_to_grid(goal_world[0], goal_world[1])

        if start_x is None or goal_x is None:
            self.get_logger().error("[PLAN] Invalid coordinates")
            return None

        self.get_logger().info(f"[PLAN] World: ({start_world[0]:.2f}, {start_world[1]:.2f}) -> ({goal_world[0]:.2f}, {goal_world[1]:.2f})")
        self.get_logger().info(f"[PLAN] Grid: ({start_x}, {start_y}) -> ({goal_x}, {goal_y})")

        # Check start point
        if not self.is_valid_cell(start_x, start_y):
            val = self.inflated_map[start_y, start_x] if 0 <= start_y < self.inflated_map.shape[0] and 0 <= start_x < self.inflated_map.shape[1] else -1
            self.get_logger().error(f"[PLAN] Start in obstacle! Grid value: {val}")
            return None

        # Check goal point
        if not self.is_valid_cell(goal_x, goal_y):
            val = self.inflated_map[goal_y, goal_x] if 0 <= goal_y < self.inflated_map.shape[0] and 0 <= goal_x < self.inflated_map.shape[1] else -1
            self.get_logger().error(f"[PLAN] Goal in obstacle! Grid value: {val}")
            return None

        # A* algorithm
        open_list = []
        closed_set = set()

        start_node = Node2D(
            start_x, start_y,
            cost=0.0,
            heuristic=self.heuristic_weight * self.heuristic_distance(start_x, start_y, goal_x, goal_y)
        )

        heapq.heappush(open_list, start_node)
        node_map = {(start_x, start_y): start_node}

        iterations = 0
        max_iterations = 50000

        while open_list and iterations < max_iterations:
            iterations += 1

            current_node = heapq.heappop(open_list)
            current_pos = (current_node.x, current_node.y)

            if current_node.x == goal_x and current_node.y == goal_y:
                self.get_logger().info(f"[PLAN] Path found in {iterations} iterations")
                path_points = self.reconstruct_path(current_node)
                
                # ========== Validate path ==========
                valid, violations = self.validate_path(path_points)
                if not valid:
                    self.get_logger().error(f"[VALIDATE] Path has {len(violations)} violations!")
                    for v in violations:
                        self.get_logger().error(
                            f"  Point {v['index']}: world={v['world']}, grid={v['grid']}, "
                            f"type={v['type']}, value={v['value']}"
                        )
                else:
                    self.get_logger().info("[VALIDATE] Path is valid - no obstacle violations")
                # ===================================

                if self.enable_smoothing and len(path_points) > 2:
                    smoothed = self.smooth_path(path_points)
                    self.get_logger().info(f"[PLAN] Smoothed: {len(path_points)} -> {len(smoothed)} points")
                    
                    # Validate smoothed path
                    valid, violations = self.validate_path(smoothed)
                    if not valid:
                        self.get_logger().error(f"[VALIDATE] Smoothed path has {len(violations)} violations!")
                        self.get_logger().warn("[VALIDATE] Returning unsmoothed path instead")
                        return path_points
                    return smoothed
                else:
                    return path_points

            closed_set.add(current_pos)

            for neighbor_x, neighbor_y, move_cost in self.get_neighbors(current_node.x, current_node.y):
                neighbor_pos = (neighbor_x, neighbor_y)

                if neighbor_pos in closed_set:
                    continue

                tentative_cost = current_node.cost + move_cost
                heuristic = self.heuristic_weight * self.heuristic_distance(
                    neighbor_x, neighbor_y, goal_x, goal_y
                )

                if neighbor_pos in node_map:
                    neighbor_node = node_map[neighbor_pos]
                    if tentative_cost < neighbor_node.cost:
                        neighbor_node.cost = tentative_cost
                        neighbor_node.parent = current_node
                        heapq.heappush(open_list, neighbor_node)
                else:
                    neighbor_node = Node2D(
                        neighbor_x, neighbor_y,
                        cost=tentative_cost,
                        heuristic=heuristic,
                        parent=current_node
                    )
                    node_map[neighbor_pos] = neighbor_node
                    heapq.heappush(open_list, neighbor_node)

        self.get_logger().error(f"[PLAN] No path found after {iterations} iterations")
        return None

    def smooth_path(self, path_points):
        """Conservative path smoothing"""
        if len(path_points) < 3:
            return path_points

        smoothed = [path_points[0]]

        i = 0
        while i < len(path_points) - 1:
            furthest = i + 1
            max_lookahead = min(i + 3, len(path_points))

            for j in range(i + 2, max_lookahead):
                start_point = path_points[i]
                test_point = path_points[j]
                distance = math.sqrt(
                    (test_point[0] - start_point[0])**2 + 
                    (test_point[1] - start_point[1])**2
                )

                if distance > 1.0:
                    break

                if self.line_of_sight(path_points[i], path_points[j]):
                    furthest = j
                else:
                    break

            smoothed.append(path_points[furthest])
            i = furthest

        return smoothed

    def line_of_sight(self, start_point, end_point):
        """Check if there's line of sight between two points"""
        x0, y0 = self.world_to_grid(start_point[0], start_point[1])
        x1, y1 = self.world_to_grid(end_point[0], end_point[1])

        if x0 is None or y0 is None or x1 is None or y1 is None:
            return False

        if not self.is_valid_cell(x0, y0) or not self.is_valid_cell(x1, y1):
            return False

        # Bresenham line algorithm
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1

        x, y = x0, y0

        if dx >= dy:
            err = dx / 2.0
            while x != x1:
                if not self.is_valid_cell(x, y):
                    return False
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
        else:
            err = dy / 2.0
            while y != y1:
                if not self.is_valid_cell(x, y):
                    return False
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy

        return self.is_valid_cell(x1, y1)

    def create_path_message(self, path_points, frame_id="map"):
        """Create ROS Path message"""
        if not path_points:
            return None

        path_msg = Path()
        path_msg.header = Header()
        path_msg.header.frame_id = frame_id
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for i, (x, y) in enumerate(path_points):
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0

            if i < len(path_points) - 1:
                next_x, next_y = path_points[i + 1]
                yaw = math.atan2(next_y - y, next_x - x)
                pose.pose.orientation.z = math.sin(yaw / 2.0)
                pose.pose.orientation.w = math.cos(yaw / 2.0)
            else:
                pose.pose.orientation.w = 1.0

            path_msg.poses.append(pose)

        return path_msg


def main(args=None):
    rclpy.init(args=args)
    planner = AStarPlanner()
    try:
        rclpy.spin(planner)
    except KeyboardInterrupt:
        pass
    finally:
        planner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()