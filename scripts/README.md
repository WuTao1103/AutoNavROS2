# Scripts Directory

This directory contains utility scripts for managing the AutoNavROS2 system.

## 🚀 System Management Scripts

### `start_full_system.sh`
**Description**: Comprehensive system launcher using tmux
- Creates multi-window tmux session
- Launches all ROS2 components with proper timing
- Includes dependency checking and error handling
- Provides detailed logging and status updates

**Usage**:
```bash
./scripts/start_full_system.sh
```

**Features**:
- Automated dependency verification
- Intelligent startup sequencing
- Real-time progress monitoring
- Comprehensive error handling

### `stop_full_system.sh`
**Description**: Graceful system shutdown script
- Safely terminates all ROS2 processes
- Cleans up tmux sessions
- Handles Gazebo and other components
- Supports force shutdown mode

**Usage**:
```bash
./scripts/stop_full_system.sh           # Graceful shutdown
./scripts/stop_full_system.sh --force   # Force shutdown
```

**Options**:
- `--force, -f`: Force immediate termination
- `--help, -h`: Show help message

### `kill_all_ros.sh`
**Description**: Emergency cleanup script
- Force kills all ROS2 and Gazebo processes
- Cleans shared memory and temporary files
- Used for recovery when normal shutdown fails

**Usage**:
```bash
./scripts/kill_all_ros.sh
```

**⚠️ Warning**: This script force-terminates processes and should only be used when normal shutdown fails.

## 🎮 Interface Scripts

### `start_rviz_slam.sh`
**Description**: Launches RViz with SLAM configuration
- Pre-configured RViz setup for navigation
- Includes map display and goal setting tools
- Optimized for SLAM visualization

**Usage**:
```bash
./scripts/start_rviz_slam.sh
```

**Prerequisites**: ROS2 system must be running first

### `start_slam_demo.sh`
**Description**: Demo script for SLAM functionality
- Simplified SLAM demonstration
- Good for testing and educational purposes
- Includes basic navigation setup

**Usage**:
```bash
./scripts/start_slam_demo.sh
```

## 🔧 Utility Scripts

### `verify_gazebo_setup.sh`
**Description**: Gazebo installation and configuration checker
- Verifies Gazebo installation
- Checks TurtleBot3 model availability
- Tests simulation environment
- Provides setup recommendations

**Usage**:
```bash
./scripts/verify_gazebo_setup.sh
```

**Output**: Detailed report of system status and recommendations

## 📝 Script Usage Guidelines

### General Usage Pattern

1. **First Time Setup**:
   ```bash
   # Check system prerequisites
   ./scripts/verify_gazebo_setup.sh

   # Start the full system
   ./scripts/start_full_system.sh
   ```

2. **Normal Operation**:
   ```bash
   # Start system (recommended - use interactive menu)
   ./quick_start.sh

   # Or start components individually
   ./scripts/start_full_system.sh
   ./scripts/start_rviz_slam.sh
   ```

3. **Shutdown**:
   ```bash
   # Graceful shutdown
   ./scripts/stop_full_system.sh

   # If normal shutdown fails
   ./scripts/kill_all_ros.sh
   ```

### Best Practices

1. **Check Prerequisites**: Always run `verify_gazebo_setup.sh` on new systems
2. **Use Interactive Menu**: `./quick_start.sh` provides the best user experience
3. **Monitor Output**: Check script output for errors or warnings
4. **Graceful Shutdown**: Always try normal shutdown before force cleanup
5. **Resource Monitoring**: Monitor system resources during operation

## 🚨 Troubleshooting

### Common Issues

| Issue | Solution | Script |
|-------|----------|--------|
| System won't start | Check dependencies | `verify_gazebo_setup.sh` |
| Processes hanging | Force cleanup | `kill_all_ros.sh` |
| RViz not opening | Check ROS2 system | `start_rviz_slam.sh` |
| Gazebo issues | Verify installation | `verify_gazebo_setup.sh` |

### Error Recovery

1. **System Hanging**:
   ```bash
   ./scripts/stop_full_system.sh --force
   ./scripts/kill_all_ros.sh
   ```

2. **Partial Startup**:
   ```bash
   ./scripts/stop_full_system.sh
   ./scripts/start_full_system.sh
   ```

3. **Permission Issues**:
   ```bash
   chmod +x scripts/*.sh
   ```

## 🔍 Script Details

### Environment Requirements
- **ROS2 Jazzy** or compatible version
- **TurtleBot3** simulation packages
- **Gazebo** for 3D simulation
- **tmux** for session management
- **Proper workspace** sourcing

### Logging
- All scripts provide detailed output
- Error messages include suggested solutions
- Status updates show progress
- Debug information available with verbose flags

### Configuration
- Scripts use project-relative paths
- Configuration parameters in script headers
- Environment variables respected
- Workspace auto-detection

## 📚 Additional Resources

- **Main Documentation**: See `README.md` in project root
- **User Guide**: `docs/user_guide.md` for detailed usage instructions
- **API Reference**: `docs/api.md` for technical details
- **Interactive Help**: Use `./quick_start.sh` for guided operation

These scripts are designed to make the AutoNavROS2 system easy to use while providing robust error handling and clear feedback to users.