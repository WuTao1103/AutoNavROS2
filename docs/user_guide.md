# User Guide - Step-by-Step Usage Instructions

Welcome to the AutoNavROS2 User Guide! This comprehensive guide will walk you through everything you need to know to successfully use the autonomous navigation system.

## 📋 Table of Contents

- [🚀 Getting Started](#-getting-started)
- [🎮 Using the Web Dashboard](#-using-the-web-dashboard)
- [🗺️ Mapping and Navigation](#️-mapping-and-navigation)
- [🎯 Setting Navigation Goals](#-setting-navigation-goals)
- [📡 Remote Control via MQTT](#-remote-control-via-mqtt)
- [🔧 Configuration and Tuning](#-configuration-and-tuning)
- [🚨 Troubleshooting](#-troubleshooting)
- [💡 Tips and Best Practices](#-tips-and-best-practices)

## 🚀 Getting Started

### Prerequisites Check

Before starting, ensure you have all required components installed:

```bash
# Check ROS2 installation
ros2 --version

# Check TurtleBot3 packages
ros2 pkg list | grep turtlebot3

# Check Node.js for dashboard
node --version
npm --version

# Check Gazebo installation
gazebo --version
```

### First Time Setup

1. **Clone and Build the Project**
   ```bash
   git clone <your-repository-url>
   cd AutoNavROS2
   colcon build
   source install/setup.bash
   ```

2. **Install Dashboard Dependencies**
   ```bash
   cd dashboard
   npm install
   cd ..
   ```

3. **Verify Installation**
   ```bash
   # Quick test - list available launch files
   ros2 launch slam <TAB><TAB>
   ros2 launch path_planning <TAB><TAB>
   ```

### Quick Start Options

#### Option 1: Automated Launch (Recommended for Beginners)
```bash
# Interactive menu system
./quick_start.sh
```

#### Option 2: Manual Launch (For Advanced Users)
```bash
# See detailed manual launch instructions in README.md
```

## 🎮 Using the Web Dashboard

### Starting the Dashboard

1. **Launch ROS2 System First**
   ```bash
   # Start simulation and navigation
   ./start_full_system.sh
   ```

2. **Start Web Dashboard**
   ```bash
   cd dashboard
   npm run dev
   ```

3. **Access Dashboard**
   - Open browser to `http://localhost:5173`
   - You should see the navigation interface

### Dashboard Overview

The dashboard consists of four main sections:

#### 🗺️ Map Visualizer (Left Panel)
- **Interactive Map**: Shows real-time occupancy grid
- **Robot Position**: Orange circle with direction arrow
- **LiDAR Data**: Red dots showing obstacle detections
- **Planned Path**: Blue/cyan line from robot to goal
- **Goal Marker**: Green circle at target location

**How to Use**:
- Click anywhere on the map to set a navigation goal
- Watch the robot's movement in real-time
- Monitor LiDAR detections for obstacle awareness

#### 🎮 Control Panel (Top Right)
- **Manual Control**: Virtual joystick for manual driving
- **Speed Controls**: Linear and angular velocity sliders
- **Emergency Stop**: Red button for immediate stop
- **Mode Display**: Current robot mode (IDLE/NAVIGATING/EMERGENCY_STOP)

**How to Use**:
- Use joystick for manual robot control
- Adjust speed sliders for different velocity limits
- Press emergency stop if needed
- Monitor current mode status

#### 📊 Status Widget (Middle Right)
- **Robot Pose**: Current X, Y coordinates and orientation
- **Velocity**: Current linear and angular speeds
- **Battery**: Power level and voltage (simulated)
- **Connection**: System connectivity status

#### 📝 Log Console (Bottom Right)
- **System Messages**: Real-time ROS node outputs
- **Error Messages**: Warnings and error notifications
- **Timestamps**: When each message occurred
- **Log Levels**: INFO, WARN, ERROR classification

## 🗺️ Mapping and Navigation

### Understanding the Mapping Process

The system uses **SLAM (Simultaneous Localization and Mapping)** to build maps:

1. **Initial State**: Robot starts with unknown environment
2. **Exploration**: LiDAR detects obstacles as robot moves
3. **Map Building**: Occupancy grid updates in real-time
4. **Localization**: Robot tracks its position on the growing map

### Map Colors and Meanings

- **Black/Dark Gray**: Walls and obstacles (occupied)
- **Light Gray**: Free/navigable space
- **Medium Gray**: Unknown/unexplored areas
- **Grid Lines**: Help visualize scale (optional)

### Navigation Process

1. **Automatic SLAM**: System continuously updates map
2. **Goal Setting**: User clicks on map or uses RViz
3. **Path Planning**: A* algorithm finds optimal route
4. **Path Following**: Pure pursuit controller executes motion
5. **Real-time Adaptation**: System responds to dynamic obstacles

## 🎯 Setting Navigation Goals

### Method 1: Web Dashboard (Recommended)

1. **Open Dashboard**: Navigate to `http://localhost:5173`
2. **Wait for Map**: Ensure map is visible and robot position shown
3. **Click Goal**: Click anywhere on the map you want robot to go
4. **Confirm Path**: Blue path line should appear
5. **Watch Navigation**: Robot will automatically navigate to goal

**Tips**:
- Choose goals in free (gray) areas, not obstacles (black)
- Avoid goals too close to walls
- Wait for current navigation to complete before setting new goals

### Method 2: RViz Interface

1. **Launch RViz**
   ```bash
   ./start_rviz_slam.sh
   ```

2. **Use 2D Nav Goal Tool**
   - Click "2D Nav Goal" button in toolbar
   - Click on map where you want robot to go
   - Drag to set desired orientation
   - Release to confirm goal

3. **Monitor Progress**
   - Watch path visualization
   - Monitor robot movement
   - Check terminal output for status

### Method 3: Command Line

```bash
# Publish goal directly to topic
ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped "
header:
  frame_id: 'map'
pose:
  position:
    x: 2.0
    y: 1.5
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0
"
```

## 📡 Remote Control via MQTT

### Setting Up MQTT Control

1. **Install MQTT Broker**
   ```bash
   sudo apt install mosquitto mosquitto-clients
   sudo systemctl start mosquitto
   sudo systemctl enable mosquitto
   ```

2. **Test MQTT Connection**
   ```bash
   # In one terminal - subscribe to status
   mosquitto_sub -h localhost -t "/robot/status/pose"

   # In another terminal - publish command
   mosquitto_pub -h localhost -t "/robot/command/velocity" \
     -m '{"linear": 0.2, "angular": 0.0}'
   ```

### Remote Control Commands

#### Velocity Control
```bash
# Move forward
mosquitto_pub -h localhost -t "/robot/command/velocity" \
  -m '{"linear": 0.5, "angular": 0.0}'

# Turn left
mosquitto_pub -h localhost -t "/robot/command/velocity" \
  -m '{"linear": 0.0, "angular": 0.5}'

# Stop
mosquitto_pub -h localhost -t "/robot/command/velocity" \
  -m '{"linear": 0.0, "angular": 0.0}'
```

#### Goal Setting
```bash
# Set navigation goal
mosquitto_pub -h localhost -t "/robot/command/goal" \
  -m '{"x": 3.0, "y": 2.0, "theta": 1.57}'
```

#### Emergency Stop
```bash
# Emergency stop
mosquitto_pub -h localhost -t "/robot/command/emergency" \
  -m '{"stop": true}'
```

### Programming Remote Control

#### Python Example
```python
import paho.mqtt.client as mqtt
import json
import time

def on_connect(client, userdata, flags, rc):
    print(f"Connected with result code {rc}")
    client.subscribe("/robot/status/pose")

def on_message(client, userdata, msg):
    data = json.loads(msg.payload.decode())
    print(f"Robot position: x={data['x']:.2f}, y={data['y']:.2f}")

# Create client
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

# Connect and start
client.connect("localhost", 1883, 60)
client.loop_start()

# Send commands
client.publish("/robot/command/velocity",
               json.dumps({"linear": 0.3, "angular": 0.0}))

time.sleep(5)

client.publish("/robot/command/velocity",
               json.dumps({"linear": 0.0, "angular": 0.0}))
```

## 🔧 Configuration and Tuning

### SLAM Configuration

Edit `src/slam/config/slam_params.yaml`:

```yaml
occupancy_grid_mapper:
  ros__parameters:
    map_width: 200              # Increase for larger areas
    map_height: 200
    map_resolution: 0.05        # Decrease for higher detail
    publish_rate: 5.0           # Increase for faster updates
    hit_probability: 0.8        # Higher = more sensitive to obstacles
    miss_probability: 0.2       # Lower = slower to clear obstacles
```

### Path Planning Configuration

Edit `src/path_planning/config/path_planning_params.yaml`:

```yaml
astar_planner:
  ros__parameters:
    inflation_radius: 0.5       # Increase for wider safety margin
    robot_radius: 0.25         # Match your robot's actual size

pure_pursuit_controller:
  ros__parameters:
    lookahead_distance: 1.5    # Increase for smoother turns
    max_linear_velocity: 0.8   # Adjust for desired speed
    max_angular_velocity: 2.0  # Adjust for turn rate
    goal_tolerance: 0.15       # Increase if having trouble reaching goals
```

### Dashboard Configuration

Edit `dashboard/constants.ts`:

```typescript
export const MAP_SIZE = 50;           // Adjust for map display size
export const MAP_RESOLUTION = 0.4;    // Meters per pixel
export const UPDATE_RATE_MS = 50;     // 20Hz for smooth animation
export const LIDAR_MAX_RANGE = 12.0;  // Match your LiDAR specs
```

## 🚨 Troubleshooting

### Common Issues and Solutions

#### Problem: Robot doesn't move
**Symptoms**: Commands sent but robot stays still

**Solutions**:
1. Check `/cmd_vel` topic:
   ```bash
   ros2 topic echo /cmd_vel
   ```
2. Verify Gazebo simulation is running
3. Check for emergency stop activation
4. Restart pure_pursuit_controller:
   ```bash
   ros2 run path_planning pure_pursuit_controller.py
   ```

#### Problem: No map displayed
**Symptoms**: Dashboard shows empty/black map area

**Solutions**:
1. Check SLAM node status:
   ```bash
   ros2 node list | grep occupancy
   ros2 topic echo /map
   ```
2. Verify LiDAR data:
   ```bash
   ros2 topic echo /scan
   ```
3. Restart SLAM system:
   ```bash
   ros2 launch slam slam.launch.py
   ```

#### Problem: Path planning fails
**Symptoms**: "Path planning failed" messages

**Solutions**:
1. Check goal location is in free space
2. Verify map is properly built
3. Increase inflation radius if robot is too close to walls
4. Check A* planner parameters

#### Problem: Robot gets lost
**Symptoms**: Robot position seems incorrect on map

**Solutions**:
1. Check TF transforms:
   ```bash
   ros2 run tf2_tools view_frames
   ```
2. Verify odometry data:
   ```bash
   ros2 topic echo /odom
   ```
3. Restart localization system
4. Consider using AMCL for better localization

#### Problem: Dashboard not loading
**Symptoms**: Browser shows errors or blank page

**Solutions**:
1. Check Node.js and npm versions
2. Reinstall dashboard dependencies:
   ```bash
   cd dashboard
   rm -rf node_modules
   npm install
   ```
3. Check browser console for errors
4. Try different browser or clear cache

### System Health Checks

#### Quick Diagnostic Commands
```bash
# Check all nodes are running
ros2 node list

# Check topic activity
ros2 topic list
ros2 topic hz /scan
ros2 topic hz /map

# Check for errors
ros2 log level get /occupancy_grid_mapper

# Monitor system resources
htop  # CPU and memory usage
```

#### Performance Monitoring
```bash
# Check node performance
ros2 run rqt_top rqt_top

# Monitor topic bandwidth
ros2 run rqt_graph rqt_graph

# Check TF performance
ros2 run rqt_tf_tree rqt_tf_tree
```

## 💡 Tips and Best Practices

### Mapping Tips
1. **Start in Open Area**: Begin mapping in a large, obstacle-free area
2. **Move Slowly**: Slow robot movement produces better maps
3. **Complete Coverage**: Drive robot through entire area to map
4. **Avoid Rapid Turns**: Smooth movements help localization
5. **Multiple Passes**: Go through areas multiple times for accuracy

### Navigation Tips
1. **Wait for Map**: Let SLAM build decent map before navigation
2. **Conservative Goals**: Choose goals well away from obstacles
3. **Monitor Progress**: Watch robot and dashboard for issues
4. **Sequential Goals**: Set one goal at a time, wait for completion
5. **Emergency Ready**: Keep emergency stop accessible

### Performance Optimization
1. **Adjust Map Resolution**: Balance detail vs. computational load
2. **Tune Update Rates**: Higher rates = smoother but more CPU usage
3. **Optimize Parameters**: Tune for your specific robot and environment
4. **Monitor Resources**: Keep CPU and memory usage reasonable
5. **Regular Restarts**: Occasionally restart system for best performance

### Safety Guidelines
1. **Emergency Stop**: Always have emergency stop ready
2. **Clear Workspace**: Ensure robot area is safe for movement
3. **Monitor Operation**: Never leave robot unattended
4. **Test in Simulation**: Verify behavior in Gazebo before real robot
5. **Gradual Speed Increase**: Start slow, increase speed as confidence grows

### Maintenance
1. **Regular Updates**: Keep ROS2 and packages updated
2. **Parameter Backups**: Save working configurations
3. **Log Monitoring**: Check logs for warnings or errors
4. **Performance Tracking**: Monitor system performance over time
5. **Documentation**: Keep notes on parameter changes and results

## 📞 Getting Help

### Debug Information Collection
When reporting issues, please provide:

1. **System Information**:
   ```bash
   ros2 --version
   lsb_release -a
   uname -a
   ```

2. **Node Status**:
   ```bash
   ros2 node list
   ros2 topic list
   ```

3. **Log Files**: Copy relevant error messages

4. **Configuration**: Share your parameter files

5. **Steps to Reproduce**: Detailed description of what you did

### Community Resources
- ROS2 Documentation: https://docs.ros.org/
- TurtleBot3 Manual: https://emanual.robotis.com/docs/en/platform/turtlebot3/
- ROS Discourse: https://discourse.ros.org/
- GitHub Issues: [Your repository]/issues

### Advanced Usage
For advanced customization and development, refer to:
- Source code in `src/` directories
- Configuration files in `config/` directories
- Launch files for system startup
- API documentation for topic/service details

This user guide should help you successfully operate the AutoNavROS2 system. Start with the basic operations and gradually explore more advanced features as you become comfortable with the system.