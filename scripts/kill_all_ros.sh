#!/bin/bash

echo "==============================================="
echo "  🛑 Cleanup all ROS2 and Gazebo processes"
echo "==============================================="

# Stop all ROS2 processes
echo "🔄 Stopping ROS2 processes..."
pkill -f ros2
pkill -f rviz2
pkill -f "python3.*ros2"

# Stop all Gazebo processes
echo "🔄 Stopping Gazebo processes..."
pkill -f gazebo
pkill -f gzserver
pkill -f gzclient

# Clean up remaining Python processes (SLAM and path planning scripts)
echo "🔄 Cleaning up SLAM and path planning processes..."
pkill -f "occupancy_grid_mapper"
pkill -f "astar_planner"
pkill -f "pure_pursuit"
pkill -f "simple_odometry"
pkill -f "interactive_goal"

# Force cleanup stubborn processes
echo "🔄 Force cleaning up stubborn processes..."
sleep 2
pkill -9 -f ros2
pkill -9 -f gazebo
pkill -9 -f rviz2

# Clean up shared memory and temporary files
echo "🔄 Cleaning up shared memory..."
sudo rm -rf /dev/shm/fastrtps_*
sudo rm -rf /tmp/.gazebo*

echo ""
echo "✅ Cleanup complete! You can now restart the system"
echo ""
echo "📋 Restart steps:"
echo "1. export TURTLEBOT3_MODEL=burger"
echo "2. ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py"
echo "3. cd /home/rog/AutoNavROS2 && source install/setup.bash"
echo "4. ros2 launch slam slam_with_planning.launch.py use_gazebo:=true"
echo "5. ./start_rviz_slam.sh"