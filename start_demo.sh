#!/usr/bin/env bash

set -e

PROJECT_DIR="$HOME/autonomous-drone-navigation"
WS="$PROJECT_DIR/drone_ws"

echo "=========================================="
echo " AUTONOMOUS DRONE NAVIGATION DEMO"
echo "=========================================="

# ROS 2
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"

cd "$PROJECT_DIR"

echo ""
echo "[1/4] Starting PX4 + Gazebo + MicroXRCEAgent..."
./start_simulation.sh

echo ""
echo "Waiting for simulation..."
sleep 8

echo ""
echo "[2/4] Starting obstacle perception..."
./scripts/start_perception.sh

sleep 3

echo ""
echo "[3/4] Starting YOLO perception..."
./start_yolo_pipeline.sh

sleep 3

echo ""
echo "[4/4] Starting Decision Layer..."

ros2 run drone_navigation_cpp decision_layer \
    > /tmp/decision_layer.log 2>&1 &

DECISION_PID=$!

echo ""
echo "=========================================="
echo " DEMO RUNNING"
echo "=========================================="
echo ""
echo "Simulation:       RUNNING"
echo "Obstacle pipeline: RUNNING"
echo "YOLO pipeline:     RUNNING"
echo "Decision Layer:    RUNNING"
echo ""
echo "Decision PID: $DECISION_PID"
echo ""
echo "Decision log:"
echo "  /tmp/decision_layer.log"
echo ""
echo "YOLO log:"
echo "  /tmp/yolo_detector.log"
echo ""
echo "Tracker log:"
echo "  /tmp/yolo_tracker.log"
echo ""
echo "=========================================="
echo " Safety priority: OBSTACLE > TARGET"
echo "=========================================="
