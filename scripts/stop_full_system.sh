#!/bin/bash
# ==============================================================================
# SLAM Navigation System Shutdown Script
# Gracefully stops all components of the TurtleBot3 SLAM navigation system
# ==============================================================================

set -e

# Configuration
SESSION_NAME="slam_navigation"
PROJECT_DIR="/home/rog/AutoNavROS2"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

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
# Graceful Shutdown Functions
# ==============================================================================

kill_gazebo_processes() {
    log "Stopping Gazebo processes..."
    pkill -f gazebo || true
    pkill -f gzserver || true
    pkill -f gzclient || true
    sleep 2
}

kill_ros_processes() {
    log "Stopping ROS2 nodes..."
    pkill -f "ros2 launch" || true
    pkill -f "ros2 run" || true
    pkill -f rviz2 || true
    sleep 2
}

kill_tmux_session() {
    if tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
        log "Killing tmux session: $SESSION_NAME"
        tmux kill-session -t "$SESSION_NAME"
        success "tmux session terminated"
    else
        warning "No tmux session '$SESSION_NAME' found"
    fi
}

cleanup_shared_memory() {
    log "Cleaning up shared memory..."
    # Clean up potential Gazebo shared memory
    sudo rm -rf /dev/shm/gazebo* 2>/dev/null || true
}

# ==============================================================================
# Main Shutdown Process
# ==============================================================================

main() {
    echo "============================================================"
    echo -e "${YELLOW}🛑 Shutting Down SLAM Navigation System${NC}"
    echo "============================================================"
    echo ""

    # Check if tmux session exists
    if ! tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
        warning "No active session '$SESSION_NAME' found"
        echo "Performing cleanup anyway..."
        echo ""
    fi

    # Send Ctrl+C to all tmux panes to gracefully stop processes
    if tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
        log "Sending interrupt signals to all processes..."

        # List all panes and send Ctrl+C
        tmux list-panes -a -F '#{session_name}:#{window_index}.#{pane_index}' | \
        grep "^$SESSION_NAME:" | \
        while read pane; do
            log "Stopping process in pane: $pane"
            tmux send-keys -t "$pane" C-c 2>/dev/null || true
        done

        # Wait for graceful shutdown
        log "Waiting for processes to terminate gracefully..."
        sleep 5
    fi

    # Force kill if necessary
    kill_ros_processes
    kill_gazebo_processes

    # Clean up tmux session
    kill_tmux_session

    # Final cleanup
    cleanup_shared_memory

    echo ""
    success "🏁 System shutdown complete!"
    echo ""
    echo -e "${BLUE}System Status:${NC}"
    echo "  ✅ All ROS2 processes stopped"
    echo "  ✅ Gazebo simulation terminated"
    echo "  ✅ tmux session closed"
    echo "  ✅ Shared memory cleaned"
    echo ""
    echo -e "${GREEN}Ready to restart with: ./start_full_system.sh${NC}"
    echo "============================================================"
}

# ==============================================================================
# Interactive Confirmation
# ==============================================================================

confirm_shutdown() {
    echo -e "${YELLOW}⚠️  This will stop all SLAM navigation processes.${NC}"
    echo ""
    read -p "Are you sure you want to continue? (y/N): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "Shutdown cancelled."
        exit 0
    fi
    echo ""
}

# ==============================================================================
# Force Mode
# ==============================================================================

force_shutdown() {
    echo "============================================================"
    echo -e "${RED}🚨 FORCE SHUTDOWN MODE${NC}"
    echo "============================================================"
    echo ""

    log "Force killing all related processes..."

    # Kill everything related to our system
    pkill -9 -f gazebo || true
    pkill -9 -f ros2 || true
    pkill -9 -f rviz2 || true
    pkill -9 -f turtlebot3 || true

    # Kill tmux session
    tmux kill-session -t "$SESSION_NAME" 2>/dev/null || true

    # Clean up
    cleanup_shared_memory

    success "Force shutdown complete!"
}

# ==============================================================================
# Script Entry Point
# ==============================================================================

# Check command line arguments
if [[ "$1" == "--force" || "$1" == "-f" ]]; then
    force_shutdown
elif [[ "$1" == "--help" || "$1" == "-h" ]]; then
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  --force, -f    Force shutdown without confirmation"
    echo "  --help, -h     Show this help message"
    echo ""
    echo "Interactive mode (default): Graceful shutdown with confirmation"
    exit 0
else
    # Interactive mode with confirmation
    confirm_shutdown
    main
fi