#!/bin/bash
# ==============================================================================
# Quick Start Helper Script
# Simple menu-driven interface for SLAM navigation system
# ==============================================================================

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
PURPLE='\033[0;35m'
NC='\033[0m'

PROJECT_DIR="/home/rog/AutoNavROS2"
SESSION_NAME="slam_navigation"

show_banner() {
    echo "============================================================"
    echo -e "${GREEN}🤖 TurtleBot3 SLAM Navigation System${NC}"
    echo -e "${CYAN}    Quick Start Menu${NC}"
    echo "============================================================"
    echo ""
}

show_menu() {
    echo -e "${YELLOW}Available Actions:${NC}"
    echo ""
    echo "  1) 🚀 Start Full System        - Launch all components with tmux"
    echo "  2) 🛑 Stop Full System         - Gracefully shutdown all components"
    echo "  3) 🔧 Build Workspace          - Build/rebuild the ROS2 workspace"
    echo "  4) 📊 System Status            - Check running processes and tmux session"
    echo "  5) 📱 Attach to Session        - Connect to running tmux session"
    echo "  6) 🔄 Restart System           - Stop and start the system"
    echo "  7) 🧹 Force Cleanup            - Force kill all processes"
    echo "  8) 📖 Show Help                - Display usage instructions"
    echo "  9) ❌ Exit"
    echo ""
}

check_tmux_session() {
    if tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
        return 0  # Session exists
    else
        return 1  # Session doesn't exist
    fi
}

build_workspace() {
    echo -e "${CYAN}🔧 Building workspace...${NC}"
    cd "$PROJECT_DIR"

    echo "Cleaning previous build..."
    rm -rf build install log

    echo "Building with colcon..."
    colcon build

    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✅ Workspace built successfully!${NC}"
    else
        echo -e "${RED}❌ Build failed!${NC}"
    fi
    echo ""
    read -p "Press Enter to continue..."
}

show_status() {
    echo -e "${CYAN}📊 System Status${NC}"
    echo "============================================================"

    # Check tmux session
    if check_tmux_session; then
        echo -e "${GREEN}✅ tmux session active:${NC} $SESSION_NAME"

        echo ""
        echo -e "${YELLOW}Active windows:${NC}"
        tmux list-windows -t "$SESSION_NAME" -F "  #{window_index}: #{window_name} (#{window_panes} panes)"

    else
        echo -e "${RED}❌ tmux session not running${NC}"
    fi

    echo ""
    echo -e "${YELLOW}ROS2 processes:${NC}"
    if pgrep -f "ros2" > /dev/null; then
        echo -e "${GREEN}✅ ROS2 processes running${NC}"
        echo "  Active nodes:"
        pgrep -f "ros2" | wc -l | xargs echo "    Count:"
    else
        echo -e "${RED}❌ No ROS2 processes found${NC}"
    fi

    echo ""
    echo -e "${YELLOW}Gazebo status:${NC}"
    if pgrep -f "gazebo" > /dev/null; then
        echo -e "${GREEN}✅ Gazebo running${NC}"
    else
        echo -e "${RED}❌ Gazebo not running${NC}"
    fi

    echo ""
    echo -e "${YELLOW}Workspace status:${NC}"
    if [ -d "$PROJECT_DIR/install" ]; then
        echo -e "${GREEN}✅ Workspace built${NC}"
    else
        echo -e "${RED}❌ Workspace not built${NC}"
    fi

    echo ""
    read -p "Press Enter to continue..."
}

attach_session() {
    if check_tmux_session; then
        echo -e "${CYAN}📱 Attaching to tmux session...${NC}"
        echo "Press Ctrl+b, then d to detach later"
        sleep 2
        tmux attach-session -t "$SESSION_NAME"
    else
        echo -e "${RED}❌ No active session found. Start the system first.${NC}"
        echo ""
        read -p "Press Enter to continue..."
    fi
}

restart_system() {
    echo -e "${CYAN}🔄 Restarting system...${NC}"

    if check_tmux_session; then
        echo "Stopping existing system..."
        ./scripts/stop_full_system.sh --force
        sleep 3
    fi

    echo "Starting fresh system..."
    ./scripts/start_full_system.sh
}

force_cleanup() {
    echo -e "${RED}🧹 Force cleanup mode${NC}"
    echo ""
    echo -e "${YELLOW}⚠️  This will force-kill all processes!${NC}"
    read -p "Are you sure? (y/N): " -n 1 -r
    echo

    if [[ $REPLY =~ ^[Yy]$ ]]; then
        ./scripts/stop_full_system.sh --force
    else
        echo "Cleanup cancelled."
    fi
    echo ""
    read -p "Press Enter to continue..."
}

show_help() {
    echo -e "${CYAN}📖 Help & Usage Instructions${NC}"
    echo "============================================================"
    echo ""
    echo -e "${YELLOW}Getting Started:${NC}"
    echo "1. First time: Build workspace (option 3)"
    echo "2. Start system (option 1)"
    echo "3. Wait 20-30 seconds for all components to start"
    echo "4. Attach to session (option 5) to see the terminals"
    echo ""
    echo -e "${YELLOW}Navigation:${NC}"
    echo "• In tmux session: Ctrl+b, then 0-4 to switch windows"
    echo "• Use RViz window: Set goals with '2D Nav Goal' tool"
    echo "• Control window: Manual control options available"
    echo ""
    echo -e "${YELLOW}Troubleshooting:${NC}"
    echo "• If system hangs: Use force cleanup (option 7)"
    echo "• If builds fail: Check ROS2 installation"
    echo "• If Gazebo crashes: Restart system (option 6)"
    echo ""
    echo -e "${YELLOW}Files:${NC}"
    echo "• Full logs: logs/session_[timestamp]/"
    echo "• Manual start: ./scripts/start_full_system.sh"
    echo "• Manual stop: ./scripts/stop_full_system.sh"
    echo ""
    read -p "Press Enter to continue..."
}

main_loop() {
    while true; do
        clear
        show_banner

        # Show quick status
        if check_tmux_session; then
            echo -e "${GREEN}🟢 System Status: RUNNING${NC}"
        else
            echo -e "${RED}🔴 System Status: STOPPED${NC}"
        fi
        echo ""

        show_menu

        read -p "Select option (1-9): " choice
        echo ""

        case $choice in
            1)
                if check_tmux_session; then
                    echo -e "${YELLOW}⚠️  System already running!${NC}"
                    echo "Use option 5 to attach or option 2 to stop first."
                    read -p "Press Enter to continue..."
                else
                    ./scripts/start_full_system.sh
                fi
                ;;
            2)
                if check_tmux_session; then
                    ./scripts/stop_full_system.sh
                else
                    echo -e "${YELLOW}ℹ️  No active session to stop.${NC}"
                    read -p "Press Enter to continue..."
                fi
                ;;
            3)
                build_workspace
                ;;
            4)
                show_status
                ;;
            5)
                attach_session
                ;;
            6)
                restart_system
                ;;
            7)
                force_cleanup
                ;;
            8)
                show_help
                ;;
            9)
                echo -e "${GREEN}👋 Goodbye!${NC}"
                exit 0
                ;;
            *)
                echo -e "${RED}❌ Invalid option. Please select 1-9.${NC}"
                sleep 1
                ;;
        esac
    done
}

# Check if we're in the right directory
cd "$PROJECT_DIR" 2>/dev/null || {
    echo -e "${RED}Error: Cannot access project directory: $PROJECT_DIR${NC}"
    exit 1
}

# Run main loop
main_loop