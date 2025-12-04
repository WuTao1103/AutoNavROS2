#!/bin/bash
# ==============================================================================
# SLAM Navigation System Launcher with tmux
# Automatically starts all required terminals for TurtleBot3 SLAM navigation
# ==============================================================================

set -e  # Exit on any error

# Configuration
SESSION_NAME="slam_navigation"
PROJECT_DIR="/home/rog/AutoNavROS2"
LOG_DIR="$PROJECT_DIR/logs/session_$(date +%Y%m%d_%H%M%S)"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Logging function
log() {
    echo -e "${CYAN}[$(date '+%H:%M:%S')]${NC} $1"
}

error() {
    echo -e "${RED}[ERROR]${NC} $1" >&2
}

success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

# ==============================================================================
# Dependency Checking Functions
# ==============================================================================

check_tmux() {
    if ! command -v tmux &> /dev/null; then
        error "tmux is not installed. Please install it:"
        echo "sudo apt update && sudo apt install tmux"
        exit 1
    fi
    log "tmux found: $(tmux -V)"
}

check_ros2() {
    if ! command -v ros2 &> /dev/null; then
        error "ROS2 is not available. Please source ROS2 setup."
        exit 1
    fi
    log "ROS2 found: $(ros2 --version)"
}

check_turtlebot3() {
    if ! ros2 pkg list | grep -q turtlebot3_gazebo; then
        error "TurtleBot3 Gazebo package not found. Please install:"
        echo "sudo apt install ros-\$ROS_DISTRO-turtlebot3-gazebo"
        exit 1
    fi
    log "TurtleBot3 packages available"
}

check_workspace() {
    if [ ! -d "$PROJECT_DIR" ]; then
        error "Project directory not found: $PROJECT_DIR"
        exit 1
    fi

    cd "$PROJECT_DIR"

    if [ ! -d "install" ]; then
        warning "Workspace not built. Building now..."
        colcon build
        if [ $? -ne 0 ]; then
            error "Failed to build workspace"
            exit 1
        fi
        success "Workspace built successfully"
    else
        log "Workspace already built"
    fi
}

check_gazebo() {
    if ! command -v gazebo &> /dev/null; then
        error "Gazebo not found. Please install Gazebo."
        exit 1
    fi
    log "Gazebo found: $(gazebo --version | head -n1)"
}

# ==============================================================================
# System Preparation
# ==============================================================================

setup_environment() {
    log "Setting up environment variables..."

    # Create log directory
    mkdir -p "$LOG_DIR"

    # Set TurtleBot3 model
    export TURTLEBOT3_MODEL=burger
    log "Set TURTLEBOT3_MODEL=$TURTLEBOT3_MODEL"

    # Source ROS2 workspace
    cd "$PROJECT_DIR"
    if [ -f "install/setup.bash" ]; then
        source install/setup.bash
        log "Sourced workspace setup"
    else
        warning "Workspace setup.bash not found"
    fi
}

# ==============================================================================
# tmux Session Management
# ==============================================================================

cleanup_existing_session() {
    if tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
        warning "Existing session '$SESSION_NAME' found. Killing it..."
        tmux kill-session -t "$SESSION_NAME"
    fi
}

create_tmux_session() {
    log "Creating tmux session: $SESSION_NAME"

    # Create new session with first window
    tmux new-session -d -s "$SESSION_NAME" -x 120 -y 30

    # Rename first window
    tmux rename-window -t "$SESSION_NAME:0" "Gazebo"

    # Create additional windows
    tmux new-window -t "$SESSION_NAME" -n "SLAM"
    tmux new-window -t "$SESSION_NAME" -n "RViz"
    tmux new-window -t "$SESSION_NAME" -n "Control"
    tmux new-window -t "$SESSION_NAME" -n "Monitor"

    # Create split panes in Monitor window for advanced terminals
    tmux select-window -t "$SESSION_NAME:Monitor"
    tmux split-window -h
    tmux split-window -v
    tmux select-pane -t 0
    tmux split-window -v

    # Rename panes in Monitor window
    tmux select-pane -t 0 -T "SLAM Mapper"
    tmux select-pane -t 1 -T "Path Planning"
    tmux select-pane -t 2 -T "Goal Bridge"
    tmux select-pane -t 3 -T "System Status"
}

# ==============================================================================
# Terminal Commands Setup
# ==============================================================================

setup_terminal_commands() {
    local base_cmd="cd $PROJECT_DIR && export TURTLEBOT3_MODEL=burger"

    # Terminal 1: Gazebo Environment
    log "Setting up Gazebo terminal..."
    tmux send-keys -t "$SESSION_NAME:Gazebo" \
        "$base_cmd" Enter \
        "echo '🚀 Starting Gazebo simulation environment...'" Enter \
        "ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py" Enter

    # Wait a moment for Gazebo to start
    sleep 3

    # Terminal 2: SLAM + Path Planning System
    log "Setting up SLAM system terminal..."
    tmux send-keys -t "$SESSION_NAME:SLAM" \
        "$base_cmd && source install/setup.bash" Enter \
        "echo '🗺️ Starting SLAM + Path Planning system...'" Enter \
        "sleep 5" Enter \
        "ros2 launch slam slam_with_planning.launch.py use_gazebo:=true" Enter

    # Terminal 3: RViz Visualization
    log "Setting up RViz terminal..."
    tmux send-keys -t "$SESSION_NAME:RViz" \
        "$base_cmd && source install/setup.bash" Enter \
        "echo '👁️ Starting RViz visualization...'" Enter \
        "sleep 8" Enter \
        "./start_rviz_slam.sh"

    # Terminal 4: Robot Control (ready but not started)
    log "Setting up Control terminal..."
    tmux send-keys -t "$SESSION_NAME:Control" \
        "$base_cmd && source install/setup.bash" Enter \
        "echo '🎮 Robot Control Terminal Ready'" Enter \
        "echo 'Available commands:'" Enter \
        "echo '  Keyboard control: ros2 run turtlebot3_teleop teleop_keyboard'" Enter \
        "echo '  High speed: ros2 run turtlebot3_teleop teleop_keyboard --ros-args -p scale_linear:=2.0 -p scale_angular:=1.5'" Enter \
        "echo '  Interactive goals: ros2 run slam interactive_goal_setter.py'" Enter \
        "echo ''" Enter

    # Advanced terminals (Monitor window)
    log "Setting up advanced monitoring terminals..."

    # Pane 0: Independent SLAM Mapper
    tmux send-keys -t "$SESSION_NAME:Monitor.0" \
        "$base_cmd && source install/setup.bash" Enter \
        "echo '🗺️ Independent SLAM Mapper (Terminal 5)'" Enter \
        "echo 'Command: ros2 run slam occupancy_grid_mapper.py'" Enter

    # Pane 1: Path Planning Service
    tmux send-keys -t "$SESSION_NAME:Monitor.1" \
        "$base_cmd && source install/setup.bash" Enter \
        "echo '🛤️ Independent Path Planning Service (Terminal 6)'" Enter \
        "echo 'Command: ros2 run path_planning path_planning_service.py'" Enter

    # Pane 2: Goal Bridge Node
    tmux send-keys -t "$SESSION_NAME:Monitor.2" \
        "$base_cmd && source install/setup.bash" Enter \
        "echo '🎯 Independent Goal Bridge Node (Terminal 7)'" Enter \
        "echo 'Command: ros2 run path_planning goal_pose_bridge.py'" Enter

    # Pane 3: System Status Monitor
    tmux send-keys -t "$SESSION_NAME:Monitor.3" \
        "$base_cmd && source install/setup.bash" Enter \
        "echo '📊 System Status Monitor'" Enter \
        "echo 'Useful commands:'" Enter \
        "echo '  ros2 node list'" Enter \
        "echo '  ros2 topic list'" Enter \
        "echo '  ros2 topic hz /scan'" Enter \
        "echo '  ros2 topic echo /cmd_vel'" Enter
}

# ==============================================================================
# Status Display
# ==============================================================================

display_session_info() {
    echo ""
    echo "============================================================"
    echo -e "${GREEN}🚀 SLAM Navigation System Started Successfully!${NC}"
    echo "============================================================"
    echo ""
    echo -e "${BLUE}tmux session:${NC} $SESSION_NAME"
    echo -e "${BLUE}Project directory:${NC} $PROJECT_DIR"
    echo -e "${BLUE}Log directory:${NC} $LOG_DIR"
    echo ""
    echo -e "${YELLOW}Available windows:${NC}"
    echo "  0: Gazebo      - 3D simulation environment"
    echo "  1: SLAM        - SLAM + Path planning system"
    echo "  2: RViz        - Visualization and navigation"
    echo "  3: Control     - Robot control options"
    echo "  4: Monitor     - Advanced monitoring (4 panes)"
    echo ""
    echo -e "${YELLOW}Quick commands:${NC}"
    echo "  Attach:        tmux attach-session -t $SESSION_NAME"
    echo "  Detach:        Ctrl+b, then d"
    echo "  Switch window: Ctrl+b, then 0-4"
    echo "  Kill session:  tmux kill-session -t $SESSION_NAME"
    echo "  Or use:        ./stop_full_system.sh"
    echo ""
    echo -e "${CYAN}🎯 Usage Instructions:${NC}"
    echo "1. Wait for all systems to start (about 20-30 seconds)"
    echo "2. Switch to RViz window (Ctrl+b, then 2)"
    echo "3. Use '2D Nav Goal' tool to set target locations"
    echo "4. Robot will automatically navigate to targets"
    echo ""
    echo "============================================================"
}

# ==============================================================================
# Main Execution
# ==============================================================================

main() {
    echo "============================================================"
    echo -e "${GREEN}🤖 TurtleBot3 SLAM Navigation System Launcher${NC}"
    echo "============================================================"
    echo ""

    log "Starting system checks..."
    check_tmux
    check_ros2
    check_turtlebot3
    check_gazebo
    check_workspace

    log "Setting up environment..."
    setup_environment

    log "Preparing tmux session..."
    cleanup_existing_session
    create_tmux_session

    log "Configuring terminal commands..."
    setup_terminal_commands

    log "System startup complete!"
    display_session_info

    # Automatically attach to session
    echo -e "${CYAN}Attaching to tmux session in 3 seconds...${NC}"
    sleep 3
    tmux attach-session -t "$SESSION_NAME"
}

# ==============================================================================
# Error Handling
# ==============================================================================

trap 'error "Script interrupted"; exit 1' INT TERM

# Run main function if script is executed directly
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi