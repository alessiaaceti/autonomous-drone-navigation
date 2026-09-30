#!/usr/bin/env bash

set -e

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source "$PROJECT_DIR/config/autonomy.env"
source "$ROS_SETUP"
source "$WS_SETUP"

echo
echo "=========================================="
echo " Autonomous Drone Navigation"
echo " System Check"
echo "=========================================="

echo
echo "[1] ROS 2"
echo "ROS_DISTRO=$ROS_DISTRO"

echo
echo "[2] ROS package"
ros2 pkg executables "$PACKAGE_NAME" 2>/dev/null | grep -E \
    "obstacle_detector|obstacle_avoidance|offboard_avoidance|camera_viewer" \
    || echo "ERROR: package executables not found"

echo
echo "[3] PX4 position topic"

if ros2 topic list | grep -qx "$PX4_POSITION_TOPIC"; then
    echo "OK: $PX4_POSITION_TOPIC"
else
    echo "ERROR: PX4 position topic not available"
fi

echo
echo "[4] Depth topic"

if ros2 topic list | grep -qx "$DEPTH_TOPIC"; then
    echo "OK: $DEPTH_TOPIC"
else
    echo "ERROR: depth topic not available"
fi

echo
echo "[5] Obstacle state"

if ros2 topic list | grep -qx "$OBSTACLE_STATE_TOPIC"; then
    echo "OK: $OBSTACLE_STATE_TOPIC"
else
    echo "ERROR: obstacle state topic not available"
fi

echo
echo "[6] Avoidance command"

if ros2 topic list | grep -qx "$AVOIDANCE_COMMAND_TOPIC"; then
    echo "OK: $AVOIDANCE_COMMAND_TOPIC"
else
    echo "ERROR: avoidance command topic not available"
fi

echo
echo "[7] MicroXRCE-DDS Agent"

if pgrep -f "MicroXRCEAgent" >/dev/null; then
    echo "OK: MicroXRCEAgent running"
else
    echo "WARNING: MicroXRCEAgent not detected"
fi

echo
echo "=========================================="
echo " System check complete"
echo "=========================================="