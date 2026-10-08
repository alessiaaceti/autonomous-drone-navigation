#!/usr/bin/env bash

echo "=========================================="
echo " STOPPING AUTONOMOUS DRONE DEMO"
echo "=========================================="

pkill -f "yolo_detector.py" 2>/dev/null || true
pkill -f "yolo_target_tracker" 2>/dev/null || true
pkill -f "decision_layer" 2>/dev/null || true
pkill -f "obstacle_detector" 2>/dev/null || true
pkill -f "obstacle_avoidance" 2>/dev/null || true
pkill -f "offboard_avoidance" 2>/dev/null || true

echo "Perception and decision nodes stopped."

echo ""
echo "If you also want to stop PX4/Gazebo:"
echo "  use the existing simulation stop command."
echo ""
echo "=========================================="
