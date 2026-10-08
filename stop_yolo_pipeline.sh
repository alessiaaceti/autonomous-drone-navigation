#!/usr/bin/env bash

echo "=========================================="
echo " STOPPING YOLO TARGET TRACKING PIPELINE"
echo "=========================================="

pkill -f "yolo_detector.py" 2>/dev/null || true
pkill -f "yolo_target_tracker" 2>/dev/null || true

echo "YOLO detector stopped."
echo "YOLO target tracker stopped."
echo "=========================================="
