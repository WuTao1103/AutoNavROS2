# SLAM Package

Simple 2D SLAM implementation for TurtleBot navigation using occupancy grid mapping.

## Overview

This package provides a basic SLAM system that creates 2D maps from LiDAR data for use with autonomous navigation. It includes:

- **Occupancy Grid Mapper**: Builds 2D maps from LiDAR scans using log-odds occupancy grid mapping
- **Simple Odometry**: Provides dead-reckoning localization (optional, use when Gazebo odom not available)
- **Integration with Path Planning**: Seamlessly works with the existing path_planning package

## Architecture

```
/scan (LiDAR) ──► Occupancy Grid Mapper ──► /map ──► Path Planning
                          ▲
/odom (Odometry) ─────────┘

TF Tree:
map ──► odom ──► base_link ──► base_scan
```

## Usage

### 1. Basic SLAM only
```bash
ros2 launch slam slam.launch.py
```

### 2. SLAM with Path Planning Integration
```bash
ros2 launch slam slam_with_planning.launch.py
```

### 3. With Gazebo Simulation
```bash
# Launch Gazebo first (TurtleBot3 world)
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Then launch SLAM with path planning
ros2 launch slam slam_with_planning.launch.py use_gazebo:=true
```

### 4. Testing without Gazebo (using map simulator)
```bash
# This will use the built-in map simulator for testing
ros2 launch slam slam_with_planning.launch.py use_gazebo:=false use_simple_odom:=true
```

## Launch Parameters

- `params_file`: Path to SLAM parameters file (default: config/slam_params.yaml)
- `use_simple_odom`: Use internal odometry node (default: false)
- `use_gazebo`: Whether running in Gazebo (affects map simulation) (default: false)

## Topics

### Subscribed Topics
- `/scan` (sensor_msgs/LaserScan): LiDAR data input
- `/odom` (nav_msgs/Odometry): Robot odometry
- `/cmd_vel` (geometry_msgs/Twist): Velocity commands (for simple odometry)

### Published Topics
- `/map` (nav_msgs/OccupancyGrid): Generated occupancy grid map
- `/odom` (nav_msgs/Odometry): Robot odometry (if using simple_odometry)

## Parameters

### Occupancy Grid Mapper
- `map_width/height`: Map dimensions in cells
- `map_resolution`: Meters per cell (default: 0.1)
- `map_origin_x/y`: Map origin in world coordinates
- `hit_probability`: Probability for occupied cells (default: 0.7)
- `miss_probability`: Probability for free cells (default: 0.3)
- `max_range`: Maximum LiDAR range to use (default: 10.0m)

## Integration with Path Planning

The SLAM system publishes maps on `/map` which are automatically used by the path planning system:

1. SLAM builds the map from sensor data
2. Path planning service uses the map for A* planning
3. Pure pursuit controller follows the planned paths
4. Robot movement creates new sensor data, updating the map

## Visualization

View the mapping process in RViz2:

```bash
ros2 run rviz2 rviz2
```

Add these displays:
- **Map**: Topic `/map`
- **LaserScan**: Topic `/scan`
- **Path**: Topic `/path` (when using with path planning)
- **TF**: To see coordinate frames

Set Fixed Frame to `map`.

## Troubleshooting

### Common Issues

1. **No map appearing**: Check that LiDAR data is being published on `/scan`
2. **Poor mapping quality**: Adjust `hit_probability` and `miss_probability` parameters
3. **TF errors**: Ensure static transforms are being published correctly
4. **Path planning not working**: Make sure SLAM is publishing maps before starting path planning

### Debug Commands

```bash
# Check topics
ros2 topic list
ros2 topic echo /map --once

# Check TF tree
ros2 run tf2_tools view_frames

# Check node status
ros2 node list
ros2 node info /occupancy_grid_mapper
```

## Testing

The package includes integration with the existing path_planning test system. Use the map simulator for testing without physical hardware:

```bash
# Start SLAM with simulated environment
ros2 launch slam slam_with_planning.launch.py

# In another terminal, test path planning
ros2 run path_planning test_path_planning_client.py
```

This will create a simulated environment with obstacles and test the complete SLAM + path planning pipeline.