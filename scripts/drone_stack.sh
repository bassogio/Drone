#!/bin/bash

# =========================================================
# Drone simulation stack helper
#
# What this script does:
# 1. Starts PX4 SITL in Gazebo
# 2. Starts MicroXRCEAgent
# 3. Starts QGroundControl
#
# It can also:
# - reset the full stack
# - reset the Gazebo world only
# - stop everything
# - restart everything
#
# =========================================================

set -e

# -----------------------------
# User settings
# -----------------------------

# Gazebo world name
WORLD_NAME="baylands"

# PX4 directory
PX4_DIR="$HOME/PX4-Autopilot"

# QGroundControl executable path
QGC_APP="$HOME/QGroundControl-x86_64.AppImage"

# XRCE DDS agent port
XRCE_PORT="8888"

# PX4 make target
PX4_TARGET="gz_x500_${WORLD_NAME}"


# -----------------------------
# Help message
# -----------------------------
show_help() {
    echo ""
    echo "Usage:"
    echo "  $0 start"
    echo "      Start PX4 SITL, MicroXRCEAgent, and QGroundControl"
    echo ""
    echo "  $0 reset"
    echo "      Fully restart the stack so the drone is spawned again"
    echo ""
    echo "  $0 world-reset"
    echo "      Reset the Gazebo world only"
    echo ""
    echo "  $0 restart"
    echo "      Same as reset: stop everything and start again"
    echo ""
    echo "  $0 stop"
    echo "      Stop PX4, Gazebo, MicroXRCEAgent, and QGroundControl"
    echo ""
    echo "Examples:"
    echo "  $0 start"
    echo "  $0 reset"
    echo "  $0 world-reset"
    echo "  $0 restart"
    echo "  $0 stop"
    echo ""
    echo "Typical workflow:"
    echo "  1. Run: $0 start"
    echo "  2. Fly the drone"
    echo "  3. If the drone crashes or gets stuck:"
    echo "       - use: $0 reset"
    echo "  4. If you only want to reset the Gazebo world:"
    echo "       - use: $0 world-reset"
    echo ""
}


# -----------------------------
# Start everything
# -----------------------------
start_stack() {
    echo "Starting PX4 SITL with Gazebo world: $WORLD_NAME"
    gnome-terminal -- bash -c "cd \"$PX4_DIR\" && make px4_sitl $PX4_TARGET; exec bash"
    sleep 4

    echo "Starting MicroXRCEAgent on UDP port $XRCE_PORT"
    gnome-terminal -- bash -c "MicroXRCEAgent udp4 -p $XRCE_PORT; exec bash"
    sleep 2

    echo "Starting QGroundControl"
    gnome-terminal -- bash -c "\"$QGC_APP\"; exec bash"
    sleep 3

    echo "All processes started"
}


# -----------------------------
# Reset Gazebo
# -----------------------------
world_reset() {
    echo "Stopping PX4 and Gazebo to reset the world"
    # Stop PX4 related processes
    pkill -f "px4" || true

    # Stop Gazebo Sim related processes
    pkill -f "gz sim" || true
    pkill -f "gz-server" || true
    pkill -f "gz-client" || true
    pkill -f "ruby $(which gz)" || true

    echo "Starting PX4 SITL with Gazebo world: $WORLD_NAME"
    gnome-terminal -- bash -c "cd \"$PX4_DIR\" && make px4_sitl $PX4_TARGET; exec bash"
}


# -----------------------------
# Stop everything
# -----------------------------
stop_stack() {
    echo "Stopping PX4, Gazebo, MicroXRCEAgent, and QGroundControl"

    # Stop PX4 related processes
    pkill -f "px4" || true

    # Stop Gazebo Sim related processes
    pkill -f "gz sim" || true
    pkill -f "gz-server" || true
    pkill -f "gz-client" || true
    pkill -f "ruby $(which gz)" || true

    # Stop XRCE agent
    pkill -f "MicroXRCEAgent" || true

    # Stop QGroundControl
    pkill -f "QGroundControl-x86_64.AppImage" || true
    pkill -f "QGroundControl" || true

    sleep 3
    echo "All processes stopped"
}


# -----------------------------
# Restart everything
# -----------------------------
restart_stack() {
    echo "Restarting full drone stack"
    stop_stack
    sleep 2
    start_stack
}


# -----------------------------
# Main command selection
# -----------------------------
case "$1" in
    start)
        start_stack
        ;;
    reset)
        restart_stack
        ;;
    world-reset)
        world_reset
        ;;
    restart)
        restart_stack
        ;;
    stop)
        stop_stack
        ;;
    help|-h|--help|"")
        show_help
        ;;
    *)
        echo "Unknown command: $1"
        show_help
        exit 1
        ;;
esac