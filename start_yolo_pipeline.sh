#!/usr/bin/env bash

set -e

PROJECT_DIR="$HOME/autonomous-drone-navigation"
WS="$PROJECT_DIR/drone_ws"

echo "=========================================="
echo " YOLO TARGET TRACKING PIPELINE"
echo "=========================================="

echo "[1/2] Starting YOLO detector..."

cd "$PROJECT_DIR"
source .venv-yolo/bin/activate

python3 yolo_detector.py > /tmp/yolo_detector.log 2>&1 &
YOLO_PID=$!

echo "YOLO started (PID: $YOLO_PID)"

sleep 3

echo "[2/2] Starting YOLO target tracker..."

source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"

ros2 run drone_navigation_cpp yolo_target_tracker > /tmp/yolo_tracker.log 2>&1 &
TRACKER_PID=$!

echo "Tracker started (PID: $TRACKER_PID)"

echo ""
echo "=========================================="
echo " PIPELINE RUNNING"
echo "=========================================="
echo "YOLO PID:     $YOLO_PID"
echo "Tracker PID:  $TRACKER_PID"
echo ""
echo "YOLO log:     /tmp/yolo_detector.log"
echo "Tracker log:  /tmp/yolo_tracker.log"
echo ""
echo "Simulation must already be running."
echo "=========================================="
