# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview
This is an autonomous TurtleBot navigation system for indoor environments using SLAM and path-planning algorithms without GPS. The project is structured as a ROS2 workspace implementing a three-tier development approach with specific team responsibilities.

## Project Requirements

### Level 1 - Foundational Sensing and Manual Navigation (Nov 10-23)
**Goal:** Robot drives safely without collisions
- Set up TurtleBot with ROS2 and verify sensor data streams
- Operate robot and log sensor data
- Implement simple reactive obstacle avoidance (e.g., turn right when sensing object < x cm)

### Level 2 - LiDAR and IMU Path Planning (Nov 24 - Dec 14)
**Goal:** Autonomous navigation with mapping
- Use LiDAR data to build 2D grid map of indoor environment
- Gather filtered IMU data using ROS2 publisher/subscriber nodes
- Integrate IMU and 2D map for path planning & obstacle avoidance

### Level 3 - 3D Semantic Mapping (Dec 9-14, Optional)
**Goal:** Advanced navigation capabilities
- Tune parameters for smoother navigation & higher accuracy
- 3D SLAM using depth camera (RealSense, Kinect)
- Semantic mapping (identify rooms, hallways, doors)
- Fleet coordination for multiple robots

### Project Responsibilities
- **Tao**: Project development and maintenance

## Build and Development Commands

### Building the workspace
```bash
# Build all packages
colcon build

# Build with symlink install for development
colcon build --symlink-install

# Build specific package
colcon build --packages-select <package_name>
```

### Running the system
```bash
# Source the workspace
source install/setup.bash

# Run SLAM only
ros2 launch slam slam.launch.py

# Run SLAM with path planning integration
ros2 launch slam slam_with_planning.launch.py

# For testing without Gazebo (using map simulator)
ros2 launch slam slam_with_planning.launch.py use_gazebo:=false use_simple_odom:=true

# Run path planning only (requires existing map)
ros2 launch path_planning simulation_test.launch.py

# Run individual nodes (examples)
ros2 run slam occupancy_grid_mapper.py
ros2 run path_planning astar_planner.py
```

## Architecture

### Package Structure
- `src/simple_avoid/` - Reactive obstacle avoidance using LiDAR data
- `src/slam/` - SLAM implementation for mapping and localization (COMPLETED - Level 2)
  - **Occupancy grid mapper**: Real-time 2D map building from LiDAR data
  - **Simple odometry**: Dead-reckoning localization for testing
  - **Integration ready**: Seamless integration with path_planning package
- `src/path_planning/` - Path planning algorithms for navigation (COMPLETED - Level 2)
  - **A* planner**: Grid-based pathfinding with obstacle avoidance
  - **Pure pursuit controller**: Path following with configurable parameters
  - **Goal pose bridge**: RViz integration with real robot position tracking
  - **Map integration**: Uses SLAM-generated maps for planning
- `configs/` - Configuration files including robot parameters
- `data/` - Data storage for maps and logs
- `docs/` - Project documentation including proposal and architecture

### Configuration
Robot parameters are configured in `configs/robot.yaml`, including:
- LiDAR topics (`/scan`)
- Velocity command topics (`/cmd_vel`)
- Motion parameters (speeds, distances, angles)
- Control loop rates

### Development Workflow
This is a ROS2 workspace following standard colcon build practices. The project implements a three-tier approach:
1. Level 1: Basic obstacle avoidance with reactive behaviors
2. Level 2: LiDAR-based 2D mapping with IMU integration for path planning
3. Level 3: Advanced 3D SLAM with semantic mapping (optional)

### Key Topics and Interfaces
- `/scan` - LiDAR sensor data input
- `/cmd_vel` - Velocity commands for robot motion
- `/map` - Occupancy grid map (published by SLAM, consumed by path planning)
- `/odom` - Robot odometry data (used for real-time position tracking)
- `/path` - Planned path from A* planner to pure pursuit controller
- `/goal_pose` - Goal position from RViz 2D Nav Goal tool
- `/plan_path` - Path planning service interface
- Configuration parameters defined in YAML files under `configs/` and `src/slam/config/`

## SLAM System (Level 2 - COMPLETED)

### SLAM Components
The SLAM system uses occupancy grid mapping with log-odds probability updates:

1. **Occupancy Grid Mapper** (`src/slam/scripts/occupancy_grid_mapper.py`)
   - Builds 2D maps from LiDAR scans using Bresenham's line algorithm
   - Configurable map size, resolution, and probability parameters
   - Real-time map updates with 2Hz publishing rate

2. **Simple Odometry** (`src/slam/scripts/simple_odometry.py`)
   - Dead-reckoning localization based on velocity commands
   - TF tree management (map→odom→base_link→base_scan)
   - Optional use when Gazebo odometry not available

3. **Integration Launch Files**
   - `slam.launch.py` - Basic SLAM functionality
   - `slam_with_planning.launch.py` - Complete SLAM + path planning system

### SLAM Usage
```bash
# Quick test of SLAM integration
./scripts/start_slam_demo.sh

# View mapping in RViz2
ros2 run rviz2 rviz2
# Add Map display with topic /map, Fixed Frame: 'map'

# Test with TurtleBot3 Gazebo
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
ros2 launch slam slam_with_planning.launch.py use_gazebo:=true
```

## Path Planning System (Level 2 - COMPLETED)

### Path Planning Components

1. **A* Path Planner** (`src/path_planning/scripts/path_planning_service.py`)
   - Grid-based A* algorithm with obstacle inflation
   - Configurable heuristics and diagonal movement
   - Real-time path planning service

2. **Pure Pursuit Controller** (`src/path_planning/scripts/pure_pursuit_controller.py`)
   - Path following with lookahead distance control
   - Velocity and steering control for smooth navigation
   - TF-based robot position tracking

3. **Goal Pose Bridge** (`src/path_planning/scripts/goal_pose_bridge.py`)
   - **UPDATED**: Now uses real robot position from `/odom` topic
   - RViz 2D Nav Goal integration with automatic path planning
   - TF coordinate transformation (odom → map frame)
   - Real-time robot position tracking for accurate path start points

### Path Planning Integration
The SLAM system provides maps to the path planning system via `/map` topic. The complete pipeline:
1. SLAM builds occupancy grid from LiDAR data
2. Goal Pose Bridge tracks robot position via `/odom` and TF transforms
3. A* planner uses the map and real robot position for obstacle-aware pathfinding
4. Pure pursuit controller follows the planned path
5. Robot motion generates new sensor data, updating the map

### RViz Integration
The system now properly integrates with RViz for interactive navigation:
- Use **2D Nav Goal** tool in RViz to set target positions
- Path planning automatically starts from robot's **current position** (not origin)
- Green path visualization shows the planned route from robot to goal
- Real-time position updates ensure accurate path planning

**See `src/slam/README.zh.md` for detailed Chinese usage instructions.**