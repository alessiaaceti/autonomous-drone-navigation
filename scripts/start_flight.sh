#!/usr/bin/env bash

set -e

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source "$PROJECT_DIR/config/autonomy.env"
source "$ROS_SETUP"
source "$WS_SETUP"

echo "=========================================="
echo " Autonomous Drone Navigation"
echo " OFFBOARD Flight Controller"
echo "=========================================="

echo
echo "WARNING:"
echo "The OFFBOARD controller can arm the simulated vehicle."
echo

gnome-terminal \
    --title="OFFBOARD Avoidance Controller" \
    -- bash -c "
        source '$ROS_SETUP'
        source '$WS_SETUP'
        ros2 run $PACKAGE_NAME $OFFBOARD_AVOIDANCE
        exec bash
    "

echo
echo "OFFBOARD controller started."