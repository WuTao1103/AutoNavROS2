# API Documentation - ROS2 Topics and Services

This document provides comprehensive information about all ROS2 topics, services, and interfaces used in the AutoNavROS2 system.

## 📋 Table of Contents

- [ROS2 Topics](#ros2-topics)
- [ROS2 Services](#ros2-services)
- [Message Types](#message-types)
- [Parameter Configuration](#parameter-configuration)
- [MQTT Interface](#mqtt-interface)
- [Node Graph](#node-graph)

## 🔗 ROS2 Topics

### Core Navigation Topics

#### `/scan` (sensor_msgs/LaserScan)
**Publisher**: TurtleBot3 LiDAR / Gazebo
**Subscriber**: occupancy_grid_mapper, astar_planner
**Description**: LiDAR sensor data for obstacle detection and mapping

```yaml
Header header
  uint32 seq
  time stamp
  string frame_id
float32 angle_min          # Start angle of scan [rad]
float32 angle_max          # End angle of scan [rad]
float32 angle_increment    # Angular distance between measurements [rad]
float32 time_increment     # Time between measurements [s]
float32 scan_time         # Time between scans [s]
float32 range_min         # Minimum range value [m]
float32 range_max         # Maximum range value [m]
float32[] ranges          # Range data [m]
float32[] intensities     # Intensity data
```

#### `/cmd_vel` (geometry_msgs/Twist)
**Publisher**: pure_pursuit_controller, manual control
**Subscriber**: TurtleBot3 base controller
**Description**: Velocity commands for robot movement

```yaml
Vector3 linear
  float64 x    # Forward velocity [m/s]
  float64 y    # Lateral velocity [m/s] (usually 0 for differential drive)
  float64 z    # Vertical velocity [m/s] (usually 0)
Vector3 angular
  float64 x    # Roll rate [rad/s] (usually 0)
  float64 y    # Pitch rate [rad/s] (usually 0)
  float64 z    # Yaw rate [rad/s]
```

#### `/odom` (nav_msgs/Odometry)
**Publisher**: TurtleBot3 base / simple_odometry
**Subscriber**: goal_pose_bridge, pure_pursuit_controller
**Description**: Robot pose and velocity estimation

```yaml
Header header
string child_frame_id
PoseWithCovariance pose
  Pose pose
    Point position
      float64 x    # X position [m]
      float64 y    # Y position [m]
      float64 z    # Z position [m]
    Quaternion orientation
      float64 x, y, z, w
  float64[36] covariance
TwistWithCovariance twist
  Twist twist
    Vector3 linear
    Vector3 angular
  float64[36] covariance
```

### SLAM Topics

#### `/map` (nav_msgs/OccupancyGrid)
**Publisher**: occupancy_grid_mapper
**Subscriber**: astar_planner, rviz, web dashboard
**Description**: 2D occupancy grid map for navigation

```yaml
Header header
MapMetaData info
  time map_load_time
  float32 resolution    # Map resolution [m/cell]
  uint32 width         # Map width [cells]
  uint32 height        # Map height [cells]
  Pose origin          # Map origin pose
int8[] data           # Occupancy data (-1: unknown, 0: free, 100: occupied)
```

### Path Planning Topics

#### `/goal_pose` (geometry_msgs/PoseStamped)
**Publisher**: RViz 2D Nav Goal, web dashboard
**Subscriber**: goal_pose_bridge
**Description**: Target goal position for navigation

```yaml
Header header
  uint32 seq
  time stamp
  string frame_id
Pose pose
  Point position
    float64 x, y, z
  Quaternion orientation
    float64 x, y, z, w
```

#### `/path` (nav_msgs/Path)
**Publisher**: path_planning_service
**Subscriber**: pure_pursuit_controller, rviz
**Description**: Planned path from current position to goal

```yaml
Header header
PoseStamped[] poses
  Header header
  Pose pose
```

## 🛠️ ROS2 Services

### Path Planning Service

#### `/plan_path` (path_planning/srv/PlanPath)
**Server**: path_planning_service
**Client**: goal_pose_bridge
**Description**: Requests path planning from start to goal position

**Request**:
```yaml
PoseStamped start     # Starting pose
PoseStamped goal      # Goal pose
float64 tolerance     # Goal tolerance [m]
```

**Response**:
```yaml
Path path            # Planned path
bool success         # Planning success flag
string message       # Status message
float64 planning_time # Time taken for planning [s]
```

## 📦 Message Types

### Custom Messages

The system primarily uses standard ROS2 messages. Custom service definitions:

#### PlanPath.srv
Located at: `src/path_planning/srv/PlanPath.srv`

```yaml
# Request
geometry_msgs/PoseStamped start
geometry_msgs/PoseStamped goal
float64 tolerance
---
# Response
nav_msgs/Path path
bool success
string message
float64 planning_time
```

## ⚙️ Parameter Configuration

### SLAM Node Parameters

#### occupancy_grid_mapper
```yaml
map_width: 200              # Map width in cells
map_height: 200             # Map height in cells
map_resolution: 0.1         # Meters per cell
map_origin_x: -10.0        # Map origin X coordinate
map_origin_y: -10.0        # Map origin Y coordinate
publish_rate: 2.0          # Map publishing rate [Hz]
hit_probability: 0.7       # Probability for occupied cells
miss_probability: 0.3      # Probability for free cells
occupied_threshold: 0.65   # Threshold for occupied classification
free_threshold: 0.35       # Threshold for free classification
max_range: 10.0           # Maximum laser range [m]
min_range: 0.1            # Minimum laser range [m]
```

### Path Planning Parameters

#### astar_planner
```yaml
inflation_radius: 0.4      # Obstacle inflation radius [m]
robot_radius: 0.2         # Robot radius for collision checking [m]
allow_diagonal: true      # Allow diagonal movement
heuristic_weight: 1.0     # A* heuristic weight
obstacle_threshold: 50    # Occupancy threshold for obstacles
debug_mode: true         # Enable debug output
```

#### pure_pursuit_controller
```yaml
lookahead_distance: 1.2   # Lookahead distance [m]
max_linear_velocity: 1.5  # Maximum linear velocity [m/s]
max_angular_velocity: 3.0 # Maximum angular velocity [rad/s]
goal_tolerance: 0.1       # Goal tolerance [m]
robot_frame: 'base_link'  # Robot frame ID
global_frame: 'odom'      # Global frame ID
```

## 📡 MQTT Interface

### Command Topics (Subscribe)

#### `/robot/command/velocity`
**Message Format**: JSON
```json
{
  "linear": 0.5,     // Linear velocity [m/s]
  "angular": 0.0     // Angular velocity [rad/s]
}
```

#### `/robot/command/goal`
**Message Format**: JSON
```json
{
  "x": 2.0,          // Goal X coordinate [m]
  "y": 1.5,          // Goal Y coordinate [m]
  "theta": 0.0       // Goal orientation [rad]
}
```

#### `/robot/command/emergency`
**Message Format**: JSON
```json
{
  "stop": true       // Emergency stop flag
}
```

### Status Topics (Publish)

#### `/robot/status/pose`
**Message Format**: JSON
```json
{
  "x": 1.2,                    // Current X position [m]
  "y": 0.8,                    // Current Y position [m]
  "theta": 1.57,               // Current orientation [rad]
  "timestamp": "2024-12-04T10:30:00Z"
}
```

#### `/robot/status/battery`
**Message Format**: JSON
```json
{
  "percentage": 85,            // Battery percentage [%]
  "voltage": 12.4,             // Battery voltage [V]
  "status": "Discharging"      // Battery status
}
```

#### `/robot/status/mode`
**Message Format**: JSON
```json
{
  "mode": "NAVIGATING",        // Robot operation mode
  "timestamp": "2024-12-04T10:30:00Z"
}
```

#### `/robot/sensor/lidar`
**Message Format**: JSON
```json
{
  "ranges": [1.2, 1.5, 2.0],  // LiDAR ranges [m]
  "angle_min": -3.14159,       // Start angle [rad]
  "angle_max": 3.14159,        // End angle [rad]
  "angle_increment": 0.0174,   // Angular resolution [rad]
  "timestamp": "2024-12-04T10:30:00Z"
}
```

## 🕸️ Node Graph

```
┌─────────────────────┐    /scan     ┌──────────────────────┐
│   TurtleBot3        │──────────────▶│ occupancy_grid_mapper│
│   • gazebo_ros      │               └──────────┬───────────┘
│   • joint_state     │                         │ /map
│   • robot_state     │               ┌─────────▼───────────┐
└──────────┬──────────┘               │   astar_planner     │
           │ /cmd_vel                 └─────────┬───────────┘
           │                                   │ /plan_path service
┌──────────▼──────────┐    /odom     ┌─────────▼───────────┐
│ pure_pursuit_       │◀──────────────│  goal_pose_bridge   │
│ controller          │               └─────────┬───────────┘
└─────────────────────┘                        │ /goal_pose
                                      ┌─────────▼───────────┐
                                      │      RViz           │
                                      │  • 2D Nav Goal      │
                                      │  • Map Display      │
                                      └─────────────────────┘
```

## 🔧 Debugging Commands

### Topic Monitoring
```bash
# List all topics
ros2 topic list

# Monitor specific topics
ros2 topic echo /scan
ros2 topic echo /odom
ros2 topic echo /map
ros2 topic echo /path

# Check topic frequency
ros2 topic hz /scan
ros2 topic hz /cmd_vel

# Topic information
ros2 topic info /scan
ros2 topic type /scan
```

### Service Testing
```bash
# List services
ros2 service list

# Call path planning service
ros2 service call /plan_path path_planning/srv/PlanPath "
start:
  header:
    frame_id: 'map'
  pose:
    position: {x: 0.0, y: 0.0, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
goal:
  header:
    frame_id: 'map'
  pose:
    position: {x: 2.0, y: 2.0, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
tolerance: 0.1
"
```

### Node Information
```bash
# List all nodes
ros2 node list

# Node information
ros2 node info /occupancy_grid_mapper
ros2 node info /astar_planner
ros2 node info /pure_pursuit_controller

# Parameter listing
ros2 param list /occupancy_grid_mapper
ros2 param get /occupancy_grid_mapper map_resolution
```

### TF Debugging
```bash
# View TF tree
ros2 run tf2_tools view_frames

# Echo transforms
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
```

## 📊 Performance Monitoring

### System Metrics
- **SLAM Update Rate**: 2 Hz (configurable)
- **Path Planning**: < 1 second for typical indoor maps
- **Control Loop**: 10 Hz
- **LiDAR Scan Rate**: 5-10 Hz (hardware dependent)

### Resource Usage
- **CPU**: Moderate (depends on map size)
- **Memory**: ~100-500 MB (depends on map resolution)
- **Network**: Low (ROS2 local communication)

## 🚨 Error Codes and Troubleshooting

### Common Error Messages

| Error | Node | Cause | Solution |
|-------|------|-------|----------|
| `No transform available` | Any | Missing TF frames | Check TF tree, restart nodes |
| `Path planning failed` | astar_planner | Goal in obstacle | Choose different goal location |
| `No laser scan received` | occupancy_grid_mapper | LiDAR not publishing | Check `/scan` topic |
| `Service not available` | goal_pose_bridge | Service server down | Restart path planning node |

### Log Levels
```bash
# Set log level for debugging
ros2 run <package> <node> --ros-args --log-level debug
```

This API documentation provides complete information for developers and users working with the AutoNavROS2 system.