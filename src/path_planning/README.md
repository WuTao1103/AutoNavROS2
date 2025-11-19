# Path Planning Package

This package implements path planning algorithms for autonomous TurtleBot navigation in indoor environments without GPS. It provides waypoint following, trajectory tracking, and integration with SLAM for autonomous navigation.

## Overview

The path planning package is part of a three-tier autonomous navigation system:
- **Level 1**: Simple reactive navigation and basic movement control
- **Level 2**: Integration with SLAM for 2D map-based path planning
- **Level 3**: Advanced 3D semantic mapping and multi-robot coordination

## Package Structure

```
path_planning/
├── CMakeLists.txt                   # CMake build configuration
├── package.xml                      # ROS2 package dependencies
├── config/
│   └── path_planning_params.yaml    # Configuration parameters
├── launch/
│   └── path_planning.launch.py      # Launch file for all nodes
├── scripts/
│   ├── simple_path_planner.py       # Basic path planning node
│   └── waypoint_follower.py         # Waypoint following controller
└── path_planning/
    └── __init__.py                   # Python package initialization
```

## Features

### Current Implementation (Level 1)
- Simple waypoint following
- Basic movement command interface
- Reactive navigation capabilities
- Configurable motion parameters

### Planned Features (Level 2)
- 2D grid map-based path planning
- Integration with SLAM localization
- Obstacle avoidance in planned paths
- Dynamic replanning capabilities

### Future Enhancements (Level 3)
- 3D path planning with depth cameras
- Semantic-aware navigation
- Multi-robot coordination
- Advanced trajectory optimization

## Dependencies

- ROS2 (Humble/Iron recommended)
- Navigation2 stack
- geometry_msgs, nav_msgs, sensor_msgs
- tf2 for coordinate transformations

## Configuration

Key parameters in `config/path_planning_params.yaml`:

- **Motion Limits**: Maximum/minimum linear and angular velocities
- **Tolerances**: Waypoint and goal reaching tolerances
- **Safety**: Obstacle distance thresholds
- **Topics**: ROS2 topic names for communication
- **Frames**: Coordinate frame definitions

## Usage

### Building the Package

```bash
# From workspace root
colcon build --packages-select path_planning

# Source the workspace
source install/setup.bash
```

### Running the Path Planner

```bash
# Launch all path planning nodes
ros2 launch path_planning path_planning.launch.py

# Run individual nodes
ros2 run path_planning simple_path_planner.py
ros2 run path_planning waypoint_follower.py
```

### Topics

#### Subscribed Topics
- `/goal_pose` (geometry_msgs/PoseStamped) - Target destination
- `/amcl_pose` (geometry_msgs/PoseWithCovarianceStamped) - Current robot pose (Level 2+)
- `/map` (nav_msgs/OccupancyGrid) - Environment map (Level 2+)

#### Published Topics
- `/cmd_vel` (geometry_msgs/Twist) - Velocity commands to robot
- `/path` (nav_msgs/Path) - Planned trajectory visualization

## Integration

### With SLAM Package
The path planner integrates with the SLAM package through:
- Map subscription for environment understanding
- Pose estimation for localization
- Coordinate frame transformations

### With Obstacle Avoidance
Coordinates with the simple_avoid package for:
- Emergency stopping capabilities
- Local obstacle detection
- Safety overrides

## Development Team

**Responsible**: Tao Wu (Path Planning Lead)
**Collaborators**:
- Vinh Vu (SLAM integration)
- Krish Gupta (Sensor integration)
- Chris Reed (Hardware interface)

## Timeline

- **Level 1** (Nov 10-23): Basic waypoint following and movement control
- **Level 2** (Nov 24-Dec 14): SLAM integration and map-based planning
- **Level 3** (Dec 9-14): Advanced features and optimization

## Contributing

1. Follow ROS2 best practices and conventions
2. Test all changes in simulation before hardware deployment
3. Update documentation for new features
4. Coordinate with SLAM team for interface changes

## License

MIT License - See project root for details.