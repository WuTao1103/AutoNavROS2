#!/bin/bash

echo "==============================================="
echo "  TurtleBot SLAM + Path Planning Demo"
echo "==============================================="
echo ""

# Check if workspace is built
if [ ! -d "install" ]; then
    echo "❌ Workspace not built, building now..."
    colcon build --packages-select slam path_planning
fi

# Source the workspace
source install/setup.bash

echo "🚀 Starting SLAM + Path Planning integrated system..."
echo ""
echo "System components:"
echo "  • Occupancy grid mapper (real-time mapping)"
echo "  • A* path planning service"
echo "  • Pure pursuit controller"
echo "  • Map simulator (for testing)"
echo ""
echo "Topic list:"
echo "  📡 /scan - LiDAR data"
echo "  🗺️ /map - Real-time map"
echo "  🎯 /path - Planned path"
echo "  🚗 /cmd_vel - Velocity control"
echo ""
echo "Interaction methods:"
echo "  1. Click '2D Nav Goal' in RViz2 to set target point"
echo "  2. Use path planning test client"
echo "  3. Publish cmd_vel commands to control robot"
echo ""

# Check if user wants to use Gazebo
read -p "Use Gazebo simulation? (y/N): " use_gazebo

if [[ $use_gazebo =~ ^[Yy]$ ]]; then
    echo ""
    echo "📋 Gazebo mode:"
    echo "Please run in another terminal first:"
    echo "  export TURTLEBOT3_MODEL=burger"
    echo "  ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py"
    echo ""
    read -p "Is Gazebo started? Press Enter to continue..."

    echo "🚀 Starting SLAM system (Gazebo mode)..."
    ros2 launch slam slam_with_planning.launch.py use_gazebo:=true
else
    echo ""
    echo "📋 Simulation mode:"
    echo "Will use built-in map simulator and simple odometry for testing"
    echo ""
    echo "🚀 Starting SLAM system (simulation mode)..."
    ros2 launch slam slam_with_planning.launch.py use_gazebo:=false use_simple_odom:=true
fi