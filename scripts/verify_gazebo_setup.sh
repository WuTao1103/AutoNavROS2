#!/bin/bash
# Quick verification of Gazebo map synchronization setup

echo "=========================================="
echo "🔍 Verifying Gazebo Map Synchronization Setup"
echo "=========================================="

# Check environment
echo ""
echo "1. Checking ROS2 environment..."
if [ -z "$ROS_DISTRO" ]; then
    echo "   ❌ ROS2 environment not loaded"
    echo "   Please run: source /opt/ros/jazzy/setup.bash"
else
    echo "   ✅ ROS2 environment: $ROS_DISTRO"
fi

# Check workspace
echo ""
echo "2. Checking workspace..."
if [ -f "install/path_planning/share/path_planning/launch/gazebo_simulation.launch.py" ]; then
    echo "   ✅ Launch file installed"
else
    echo "   ❌ Launch file not found"
    echo "   Please run: colcon build --packages-select path_planning"
fi

# Check executables
echo ""
echo "3. Checking executables..."
if [ -f "install/path_planning/lib/path_planning/gazebo_map_sync.py" ]; then
    echo "   ✅ gazebo_map_sync.py installed"
else
    echo "   ❌ gazebo_map_sync.py not found"
fi

if [ -f "install/path_planning/lib/path_planning/simple_map_simulator.py" ]; then
    echo "   ✅ simple_map_simulator.py installed"
else
    echo "   ❌ simple_map_simulator.py not found"
fi

# Check package dependencies
echo ""
echo "4. Checking package dependencies..."
if ros2 pkg list | grep -q "gazebo_msgs"; then
    echo "   ✅ gazebo_msgs package available"
else
    echo "   ⚠️  gazebo_msgs package not installed"
    echo "   Please run: sudo apt install ros-$ROS_DISTRO-gazebo-msgs"
fi

# Check launch file parameters
echo ""
echo "5. Testing launch file..."
if source install/setup.bash 2>/dev/null && ros2 launch path_planning gazebo_simulation.launch.py --show-args > /dev/null 2>&1; then
    echo "   ✅ Launch file can be loaded normally"
    echo ""
    echo "   Available parameters:"
    ros2 launch path_planning gazebo_simulation.launch.py --show-args 2>/dev/null | grep -A 10 "Arguments"
else
    echo "   ❌ Launch file loading failed"
    echo "   Please ensure you have run: source install/setup.bash"
fi

echo ""
echo "=========================================="
echo "📋 Usage Instructions:"
echo "=========================================="
echo ""
echo "Terminal 1 - Start Gazebo:"
echo "  export TURTLEBOT3_MODEL=burger"
echo "  ros2 launch turtlebot3_gazebo empty_world.launch.py"
echo ""
echo "Terminal 2 - Start map synchronization:"
echo "  cd $(pwd)"
echo "  source /opt/ros/jazzy/setup.bash"
echo "  source install/setup.bash"
echo "  ros2 launch path_planning gazebo_simulation.launch.py map_type:=room_with_obstacles"
echo ""
echo "=========================================="

