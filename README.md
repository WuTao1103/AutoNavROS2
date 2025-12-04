# AutoNavROS2 🤖

A comprehensive indoor autonomous TurtleBot navigation system using SLAM and path planning algorithms without GPS. This project combines ROS2 backend, real-time web visualization, and MQTT-based remote control capabilities.

<div align="center">

![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue.svg)
![Python](https://img.shields.io/badge/Python-3.8+-green.svg)
![TypeScript](https://img.shields.io/badge/TypeScript-5.8+-blue.svg)
![React](https://img.shields.io/badge/React-19.2-61DAFB.svg)
![License](https://img.shields.io/badge/License-MIT-yellow.svg)

</div>

## 📋 Table of Contents

- [🌟 Features](#-features)
- [🏗️ System Architecture](#️-system-architecture)
- [🚀 Quick Start](#-quick-start)
- [🎮 Web Dashboard](#-web-dashboard)
- [📡 MQTT Control Interface](#-mqtt-control-interface)
- [📦 Project Structure](#-project-structure)
- [⚙️ Configuration Parameters](#️-configuration-parameters)
- [🛠️ Development Guide](#️-development-guide)
- [📖 Related Documentation](#-related-documentation)

## 🌟 Features

### Core Navigation System
- **🗺️ Real-time SLAM Mapping**: Occupancy grid mapping using LiDAR data
- **🧭 Path Planning**: A* algorithm with obstacle inflation and path smoothing
- **🎯 Goal Setting**: Interactive goal setting via RViz or Web interface
- **🚗 Pure Pursuit Control**: Smooth path following with configurable control parameters
- **🛡️ Safety System**: Collision detection and emergency stop functionality

### Web Visualization Dashboard
- **📊 Real-time Map Visualization**: Interactive 2D occupancy grid map display
- **🤖 Robot Status Monitoring**: Real-time pose, velocity, and battery status
- **📈 Sensor Data Display**: LiDAR point cloud and path visualization
- **🎮 Manual Control**: Joystick-like robot control interface
- **📝 System Logs**: Real-time ROS node status and debugging information

### Remote Control & Integration
- **📡 MQTT Integration**: Remote control via MQTT broker
- **🌐 Web API**: RESTful API for external system integration
- **📱 Multi-platform Access**: Web-based interface accessible from any device
- **🔄 Real-time Synchronization**: Real-time data flow between ROS2 and Web interface

## 🏗️ System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                     Web Dashboard                            │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────┐ │
│  │Map Renderer │  │Control Panel│  │  Status & Logs      │ │
│  │• 2D Map     │  │• Manual Ctrl│  │  • Robot Status     │ │
│  │• Robot Pose │  │• Goal Set   │  │  • System Logs      │ │
│  │• LiDAR Data │  │• Emergency  │  │  • Sensor Data     │ │
│  └─────────────┘  └─────────────┘  └─────────────────────┘ │
└─────────────────┬───────────────────────────────────────────┘
                  │ WebSocket/HTTP
                  ▼
         ┌─────────────────┐
         │   MQTT Broker   │
         │ (Eclipse        │
         │  Mosquitto)     │
         └─────────┬───────┘
                   │ MQTT Topics:
                   │ • /robot/command/*
                   │ • /robot/status/*
                   │ • /robot/sensor/*
                   ▼
    ┌─────────────────────────────────────┐
    │         ROS2 Backend System         │
    │  ┌─────────────┐  ┌───────────────┐ │
    │  │MQTT-ROS     │  │  SLAM System  │ │
    │  │Bridge       │  │  • Mapping    │ │
    │  │• Topic      │◄─┤  • Localization│ │
    │  │  Conversion │  │               │ │
    │  │• Data Sync  │  └───────────────┘ │
    │  └─────────────┘                    │
    │  ┌─────────────┐  ┌───────────────┐ │
    │  │Path Planning│  │  Controller   │ │
    │  │• A* Planner │  │  • Pure Pursuit│ │
    │  │• Goal Bridge│  │  • Safety Ctrl │ │
    │  │             │  │               │ │
    │  └─────────────┘  └───────────────┘ │
    └─────────────┬───────────────────────┘
                  │ ROS2 Topics & Services:
                  │ • /scan, /cmd_vel, /odom
                  │ • /map, /path, /goal_pose
                  ▼
         ┌─────────────────┐
         │   TurtleBot3    │
         │ • LiDAR Sensor  │
         │ • IMU/Odometry  │
         │ • Chassis Control│
         └─────────────────┘
```

### Three-Tier Development Approach

1. **Level 1 - Foundation Layer** ✅
   - Basic obstacle avoidance using LiDAR
   - Safe manual navigation
   - ROS2 infrastructure setup

2. **Level 2 - Autonomous Navigation** ✅
   - SLAM-based mapping and localization
   - A* path planning with obstacle avoidance
   - Pure pursuit path following

3. **Level 3 - Advanced Interface** ✅
   - Web-based visualization dashboard
   - MQTT remote control integration
   - Real-time monitoring and logging

## 🚀 Quick Start

### System Requirements

- **ROS2 Jazzy** or higher
- **TurtleBot3 simulation packages**
- **Node.js 18+** (for dashboard)
- **Python 3.8+**
- **Gazebo** (for simulation)

### Installation Steps

1. **Clone the repository:**
   ```bash
   git clone https://github.com/WuTao1103/AutoNavROS2.git
   cd AutoNavROS2
   ```

2. **Build ROS2 workspace:**
   ```bash
   colcon build
   source install/setup.bash
   ```

3. **Install dashboard dependencies:**
   ```bash
   cd dashboard
   npm install
   cd ..
   ```

### Launching the System

#### Method 1: Interactive Menu (Recommended)
```bash
./quick_start.sh
```
The interactive menu provides guided system management:
- System status monitoring
- Automatic dependency checking
- Step-by-step launch process
- Built-in troubleshooting tools

#### Method 2: Automated Scripts
```bash
# Launch full system using tmux
./scripts/start_full_system.sh

# Launch Web dashboard separately
cd dashboard && npm run dev

# Launch RViz visualization (optional)
./scripts/start_rviz_slam.sh
```

#### Method 3: Manual Launch (Advanced Users)
```bash
# Terminal 1: Launch Gazebo simulation
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Terminal 2: Launch SLAM + Path Planning
source install/setup.bash
ros2 launch slam slam_with_planning.launch.py use_gazebo:=true

# Terminal 3: Launch Web dashboard
cd dashboard && npm run dev

# Terminal 4: Launch RViz (optional)
./scripts/start_rviz_slam.sh
```

## 🎮 Web Dashboard

The Web dashboard provides a comprehensive interface for monitoring and controlling the robot system.

### Features

- **🗺️ Interactive Map**: Real-time occupancy grid with click-to-navigate
- **📊 Status Monitoring**: Robot pose, velocity, battery, and connection status
- **🕹️ Manual Control**: Virtual joystick for manual robot operation
- **📈 Sensor Visualization**: Real-time LiDAR point cloud display
- **🎯 Goal Setting**: Click on map to set navigation goals
- **📜 System Logs**: Real-time ROS node messages and debugging information
- **🚨 Emergency Control**: Immediate stop and safety override

### Access

```bash
cd dashboard
npm run dev
# Open http://localhost:5173 in your browser
```

### Dashboard Components

#### MapVisualizer
- Canvas-based 2D map rendering
- Real-time robot pose and orientation
- LiDAR scan point cloud visualization
- Interactive goal setting via mouse click
- Path visualization with waypoints

#### ControlPanel
- Manual velocity control (linear/angular)
- Emergency stop functionality
- Mode switching (Idle/Navigation/Emergency Stop)
- Quick action buttons

#### StatusWidget
- Real-time robot status display
- Battery status and voltage
- Connection status indicator
- Current operation mode

#### LogConsole
- Real-time system logs from ROS nodes
- Filter by log level (INFO/WARN/ERROR)
- Timestamped messages with node names
- Auto-scroll and buffer management

## 📡 MQTT Control Interface

### Architecture Design

The system supports MQTT-based remote control for IoT integration and external system communication.

```
MQTT Broker (Eclipse Mosquitto)
├── /robot/command/velocity     # Velocity commands
├── /robot/command/goal         # Goal position setting
├── /robot/command/emergency    # Emergency stop
├── /robot/status/pose          # Robot position feedback
├── /robot/status/battery       # Battery status
├── /robot/status/mode          # Operation mode
└── /robot/sensor/lidar         # LiDAR data stream
```

### MQTT Topics

#### Command Topics (Subscribe)
- **`/robot/command/velocity`**: Send velocity commands
  ```json
  {"linear": 0.5, "angular": 0.0}
  ```

- **`/robot/command/goal`**: Set navigation goal
  ```json
  {"x": 2.0, "y": 1.5, "theta": 0.0}
  ```

- **`/robot/command/emergency`**: Emergency stop
  ```json
  {"stop": true}
  ```

#### Status Topics (Publish)
- **`/robot/status/pose`**: Current robot position
  ```json
  {"x": 1.2, "y": 0.8, "theta": 1.57, "timestamp": "2024-12-04T10:30:00Z"}
  ```

- **`/robot/status/battery`**: Battery information
  ```json
  {"percentage": 85, "voltage": 12.4, "status": "Discharging"}
  ```

- **`/robot/status/mode`**: Operation mode
  ```json
  {"mode": "NAVIGATING", "timestamp": "2024-12-04T10:30:00Z"}
  ```

### Setting Up MQTT Integration

1. **Install MQTT broker:**
   ```bash
   sudo apt install mosquitto mosquitto-clients
   sudo systemctl start mosquitto
   ```

2. **Configure MQTT bridge:**
   ```bash
   # Edit mqtt_bridge configuration
   nano src/mqtt_bridge/config/mqtt_config.yaml
   ```

3. **Launch with MQTT support:**
   ```bash
   ros2 launch slam slam_with_planning.launch.py use_mqtt:=true
   ```

### Remote Control Examples

#### Python Client
```python
import paho.mqtt.client as mqtt
import json

client = mqtt.Client()
client.connect("localhost", 1883, 60)

# Send velocity command
velocity_cmd = {"linear": 0.5, "angular": 0.2}
client.publish("/robot/command/velocity", json.dumps(velocity_cmd))

# Set navigation goal
goal = {"x": 3.0, "y": 2.0, "theta": 0.0}
client.publish("/robot/command/goal", json.dumps(goal))
```

#### Node.js Client
```javascript
const mqtt = require('mqtt');
const client = mqtt.connect('mqtt://localhost:1883');

// Subscribe to robot status
client.subscribe('/robot/status/pose');
client.on('message', (topic, message) => {
  const data = JSON.parse(message.toString());
  console.log(`Robot position: x=${data.x}, y=${data.y}`);
});

// Send command
client.publish('/robot/command/velocity',
  JSON.stringify({linear: 0.3, angular: 0.0}));
```

## 📦 Project Structure

```
AutoNavROS2/
├── README.md                    # This file
├── dashboard/                   # Web visualization interface
│   ├── App.tsx                 # React main application
│   ├── components/             # React components
│   │   ├── MapVisualizer.tsx   # 2D map renderer
│   │   ├── ControlPanel.tsx    # Robot control interface
│   │   ├── StatusWidget.tsx    # Status display
│   │   └── LogConsole.tsx      # System logs
│   ├── services/               # Business logic
│   │   └── rosSimulation.ts    # ROS simulation service
│   ├── types.ts                # TypeScript type definitions
│   ├── constants.ts            # Configuration constants
│   └── package.json            # Node.js dependencies
├── src/                        # ROS2 packages
│   ├── slam/                   # SLAM package
│   │   ├── scripts/
│   │   │   ├── occupancy_grid_mapper.py    # Grid mapping
│   │   │   ├── simple_odometry.py          # Odometry estimation
│   │   │   └── interactive_goal_setter.py  # Manual goal setting
│   │   ├── launch/
│   │   │   └── slam_with_planning.launch.py # System launcher
│   │   └── config/
│   │       └── slam_params.yaml            # SLAM parameters
│   ├── path_planning/          # Path planning package
│   │   ├── scripts/
│   │   │   ├── astar_planner.py           # A* algorithm
│   │   │   ├── pure_pursuit_controller.py # Path follower
│   │   │   ├── goal_pose_bridge.py        # Goal interface
│   │   │   └── path_planning_service.py   # Planning service
│   │   ├── srv/
│   │   │   └── PlanPath.srv               # Service definition
│   │   └── config/
│   │       └── path_planning_params.yaml  # Planning parameters
│   └── mqtt_bridge/            # MQTT integration
│       ├── scripts/
│       │   └── mqtt_ros_bridge.py         # MQTT-ROS bridge
│       └── config/
│           └── mqtt_config.yaml           # MQTT settings
├── configs/                    # System configuration
│   └── robot.yaml             # Robot parameters
├── scripts/                    # Utility scripts
│   ├── start_full_system.sh   # Auto launcher
│   ├── stop_full_system.sh    # System shutdown
│   ├── start_rviz_slam.sh     # RViz launcher
│   ├── start_slam_demo.sh     # SLAM demo
│   ├── kill_all_ros.sh        # Emergency cleanup
│   └── verify_gazebo_setup.sh # System verification
└── docs/                      # Documentation
    ├── api.md                 # API documentation
    └── user_guide.md          # User guide
```

## ⚙️ Configuration Parameters

### SLAM Parameters (`src/slam/config/slam_params.yaml`)
```yaml
occupancy_grid_mapper:
  ros__parameters:
    map_width: 200              # Grid count
    map_height: 200             # Grid count
    map_resolution: 0.1         # meters/grid
    publish_rate: 2.0           # Hz
    hit_probability: 0.7        # Occupied grid probability
    miss_probability: 0.3       # Free grid probability
    max_range: 10.0            # Maximum laser range
```

### Path Planning Parameters (`src/path_planning/config/path_planning_params.yaml`)
```yaml
astar_planner:
  ros__parameters:
    inflation_radius: 0.4       # Obstacle inflation radius (meters)
    robot_radius: 0.2          # Robot radius (meters)
    allow_diagonal: true       # Enable diagonal movement
    heuristic_weight: 1.0      # A* heuristic weight

pure_pursuit_controller:
  ros__parameters:
    lookahead_distance: 1.2    # Lookahead distance (meters)
    max_linear_velocity: 1.5   # Maximum linear velocity (m/s)
    max_angular_velocity: 3.0  # Maximum angular velocity (rad/s)
    goal_tolerance: 0.1        # Goal tolerance (meters)
```

### Dashboard Configuration (`dashboard/constants.ts`)
```typescript
export const MAP_SIZE = 40;           // Map grid size
export const MAP_RESOLUTION = 0.5;    // meters/pixel
export const UPDATE_RATE_MS = 50;     // 20Hz update rate
export const LIDAR_RAYS = 360;        // LiDAR resolution
export const LIDAR_MAX_RANGE = 10.0;  // Maximum range
```

## 🛠️ Development Guide

### Adding New Features

1. **ROS2 Node**: Add new node in `src/[package]/scripts/`
2. **Web Component**: Create new React component in `dashboard/components/`
3. **MQTT Topic**: Define new topic in MQTT bridge configuration
4. **Launch File**: Update launch files in `src/[package]/launch/`

### Testing

```bash
# Test ROS2 packages
colcon test --packages-select slam path_planning

# Test Web dashboard
cd dashboard
npm test

# Integration tests
./scripts/run_integration_tests.sh
```

### Production Build

```bash
# Build ROS2 workspace
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Build dashboard
cd dashboard
npm run build

# Create deployment package
./scripts/create_deployment.sh
```

## 📖 Related Documentation

- **[Chinese Documentation](README.zh.md)**: Chinese version of this README
- **[API Documentation](docs/api.md)**: ROS2 topics and services
- **[User Guide](docs/user_guide.md)**: Step-by-step usage instructions

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments

- **Wu Tao**: Project development and maintenance
- **ROS2 Community**: Framework and tools
- **TurtleBot3**: Robot platform and simulation
- **React/TypeScript**: Web interface technologies

---

<div align="center">
<p>Built with ❤️ using ROS2, React, and modern web technologies</p>

**🌟 Please give us a Star if you find this useful!**
</div>
