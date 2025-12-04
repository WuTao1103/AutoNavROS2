#!/bin/bash

echo "==============================================="
echo "  Starting RViz2 Visualization Interface"
echo "==============================================="
echo ""

# Source the workspace
source install/setup.bash

echo "🎮 Starting RViz2..."
echo ""
echo "📋 Configuration Steps:"
echo "  1. Set Fixed Frame to: map"
echo "  2. Add the following displays:"
echo "     • Map (topic: /map)"
echo "     • LaserScan (topic: /scan)"
echo "     • Path (topic: /path)"
echo "     • RobotModel"
echo "     • TF"
echo ""
echo "🎯 Interaction:"
echo "  • Click '2D Nav Goal' in the toolbar"
echo "  • Click on the map to set target point"
echo "  • Robot will automatically plan path and move"
echo ""

# Check if config file exists
if [ -f "rviz_slam_config.rviz" ]; then
    echo "📁 Starting RViz2 with preset configuration..."
    ros2 run rviz2 rviz2 -d rviz_slam_config.rviz
else
    echo "📁 Starting RViz2 with default configuration..."
    ros2 run rviz2 rviz2
fi