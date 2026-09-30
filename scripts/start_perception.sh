#!/usr/bin/env bash

set -e

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source "$PROJECT_DIR/config/autonomy.env"
source "$ROS_SETUP"
source "$WS_SETUP"

open_terminal()
{
    local title="$1"
    local command="$2"

    gnome-terminal \
        --title="$title" \
        -- bash -c "
            source '$ROS_SETUP'
            source '$WS_SETUP'
            $command
            exec bash
        "
}

echo "=========================================="
echo " Autonomous Drone Navigation"
echo " Perception Pipeline"
echo "=========================================="

open_terminal \
    "Obstacle Detector" \
    "ros2 run $PACKAGE_NAME $OBSTACLE_DETECTOR"

open_terminal \
    "Obstacle Avoidance" \
    "ros2 run $PACKAGE_NAME $OBSTACLE_AVOIDANCE"

echo
echo "Perception pipeline started."