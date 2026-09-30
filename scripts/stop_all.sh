#!/usr/bin/env bash

set -e

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source "$PROJECT_DIR/config/autonomy.env"

echo "=========================================="
echo " Stopping Autonomous Drone Navigation"
echo "=========================================="

pkill -f "obstacle_detector" 2>/dev/null || true
pkill -f "obstacle_avoidance" 2>/dev/null || true
pkill -f "offboard_avoidance" 2>/dev/null || true
pkill -f "camera_viewer" 2>/dev/null || true

"$SIMULATION_SCRIPT" stop 2>/dev/null || true

echo
echo "All project processes stopped."