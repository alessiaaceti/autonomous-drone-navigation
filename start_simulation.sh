#!/usr/bin/env bash

set -u

# ============================================================
# Autonomous Drone Navigation - Full Simulation Launcher
# PX4 + Gazebo + QGroundControl + MicroXRCEAgent
# + ROS 2 Camera + Perception
#
# Usage:
#   ./start_simulation.sh
#   ./start_simulation.sh stop
#   ./start_simulation.sh restart
# ============================================================

PX4_DIR="$HOME/PX4-Autopilot"
ROS_WS="$HOME/autonomous-drone-navigation/drone_ws"
QGC_APP="$HOME/Downloads/QGroundControl.AppImage"

GZ_CONFIG_PATH_VALUE="/usr/share/gz:/opt/ros/jazzy/opt/gz_transport_vendor/share/gz:/opt/ros/jazzy/opt/gz_msgs_vendor/share/gz"

CAMERA_TOPIC="/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image"


# ============================================================
# STOP
# ============================================================

stop_simulation() {

    echo ""
    echo "============================================================"
    echo "       STOPPING AUTONOMOUS DRONE NAVIGATION"
    echo "============================================================"
    echo ""

    echo "[1/6] Stopping ROS 2 perception..."

    pkill -f "ros2 run drone_navigation_cpp camera_viewer" 2>/dev/null || true


    echo "[2/6] Stopping ROS 2 camera bridge..."

    pkill -f "ros2 run ros_gz_bridge parameter_bridge" 2>/dev/null || true
    pkill -f "parameter_bridge.*IMX214" 2>/dev/null || true


    echo "[3/6] Stopping MicroXRCEAgent..."

    pkill -f "MicroXRCEAgent udp4 -p 8888" 2>/dev/null || true


    echo "[4/6] Stopping PX4 + Gazebo..."

    pkill -f "make px4_sitl gz_x500_depth" 2>/dev/null || true
    pkill -f "px4_sitl_default/bin/px4" 2>/dev/null || true
    pkill -f "gz sim" 2>/dev/null || true


    echo "[5/6] Stopping QGroundControl..."

    pkill -f "QGroundControl.AppImage" 2>/dev/null || true


    echo "[6/6] Closing simulation terminals..."

    pkill -f "gnome-terminal.*PX4 + Gazebo" 2>/dev/null || true
    pkill -f "gnome-terminal.*QGroundControl" 2>/dev/null || true
    pkill -f "gnome-terminal.*MicroXRCEAgent" 2>/dev/null || true
    pkill -f "gnome-terminal.*ROS 2 Camera Bridge" 2>/dev/null || true
    pkill -f "gnome-terminal.*ROS 2 Perception" 2>/dev/null || true

    sleep 2

    echo ""
    echo "============================================================"
    echo " Simulation stopped."
    echo "============================================================"
    echo ""
}


# ============================================================
# START
# ============================================================

start_simulation() {

    echo ""
    echo "============================================================"
    echo "       AUTONOMOUS DRONE NAVIGATION"
    echo "============================================================"
    echo "  PX4 + Gazebo + QGroundControl + MicroXRCEAgent"
    echo "  + ROS 2 Perception"
    echo "============================================================"
    echo ""

    # --------------------------------------------------------
    # Basic checks
    # --------------------------------------------------------

    if ! command -v gnome-terminal >/dev/null 2>&1; then
        echo "ERROR: gnome-terminal is not installed."
        exit 1
    fi

    if [ ! -d "$PX4_DIR" ]; then
        echo "ERROR: PX4 directory not found:"
        echo "  $PX4_DIR"
        exit 1
    fi

    if [ ! -d "$ROS_WS/install" ]; then
        echo "ERROR: ROS 2 workspace is not built:"
        echo "  $ROS_WS"
        exit 1
    fi

    if ! command -v MicroXRCEAgent >/dev/null 2>&1; then
        echo "ERROR: MicroXRCEAgent not found."
        exit 1
    fi

    if ! command -v ros2 >/dev/null 2>&1; then
        echo "ERROR: ROS 2 command not found."
        exit 1
    fi

    if [ ! -f "$QGC_APP" ]; then
        echo "ERROR: QGroundControl AppImage not found:"
        echo "  $QGC_APP"
        exit 1
    fi

    echo "[OK] PX4 found"
    echo "[OK] ROS 2 workspace found"
    echo "[OK] MicroXRCEAgent found"
    echo "[OK] ROS 2 found"
    echo "[OK] QGroundControl found"
    echo "[OK] Gazebo configuration ready"
    echo ""


    # --------------------------------------------------------
    # 1. PX4 + Gazebo
    # --------------------------------------------------------

    echo "[1/5] Starting PX4 + Gazebo..."

    gnome-terminal --title="PX4 + Gazebo" -- bash -c "
        echo '=========================================='
        echo ' PX4 + GAZEBO'
        echo '=========================================='
        echo ''
        
        export GZ_CONFIG_PATH='$GZ_CONFIG_PATH_VALUE'

        cd '$PX4_DIR'

        echo 'Starting PX4 SITL with X500 depth camera...'
        echo ''

        make px4_sitl gz_x500_depth

        EXIT_CODE=\$?

        echo ''
        echo 'PX4/Gazebo terminated with exit code: \$EXIT_CODE'
        read -p 'Press ENTER to close this terminal...'
    "

    echo "Waiting for PX4/Gazebo to initialize..."
    sleep 8


    # --------------------------------------------------------
    # 2. QGroundControl
    # --------------------------------------------------------

    echo "[2/5] Starting QGroundControl..."

    gnome-terminal --title="QGroundControl" -- bash -c "
        echo '=========================================='
        echo ' QGROUNDCONTROL'
        echo '=========================================='
        echo ''
        echo 'Starting QGroundControl...'
        echo ''

        '$QGC_APP'

        EXIT_CODE=\$?

        echo ''
        echo 'QGroundControl terminated with exit code: \$EXIT_CODE'
        read -p 'Press ENTER to close this terminal...'
    "

    echo "Waiting for QGroundControl to initialize..."
    sleep 5


    # --------------------------------------------------------
    # 3. MicroXRCEAgent
    # --------------------------------------------------------

    echo "[3/5] Starting MicroXRCEAgent..."

    gnome-terminal --title="MicroXRCEAgent" -- bash -c "
        echo '=========================================='
        echo ' MICROXRCE-DDS AGENT'
        echo '=========================================='
        echo ''
        echo 'Listening on UDP port 8888...'
        echo ''

        MicroXRCEAgent udp4 -p 8888

        EXIT_CODE=\$?

        echo ''
        echo 'MicroXRCEAgent terminated with exit code: \$EXIT_CODE'
        read -p 'Press ENTER to close this terminal...'
    "

    sleep 2


    # --------------------------------------------------------
    # 4. Gazebo -> ROS 2 Camera Bridge
    # --------------------------------------------------------

    echo "[4/5] Starting ROS 2 Camera Bridge..."

    gnome-terminal --title="ROS 2 Camera Bridge" -- bash -c "
        echo '=========================================='
        echo ' GAZEBO -> ROS 2 CAMERA BRIDGE'
        echo '=========================================='
        echo ''

        source /opt/ros/jazzy/setup.bash
        source '$ROS_WS/install/setup.bash'

        export GZ_CONFIG_PATH='$GZ_CONFIG_PATH_VALUE'

        echo 'Bridging camera topic:'
        echo '$CAMERA_TOPIC'
        echo ''

        ros2 run ros_gz_bridge parameter_bridge \
            '$CAMERA_TOPIC@sensor_msgs/msg/Image@gz.msgs.Image'

        EXIT_CODE=\$?

        echo ''
        echo 'Camera bridge terminated with exit code: \$EXIT_CODE'
        read -p 'Press ENTER to close this terminal...'
    "

    sleep 3


    # --------------------------------------------------------
    # 5. ROS 2 Perception
    # --------------------------------------------------------

    echo "[5/5] Starting ROS 2 Perception..."

    gnome-terminal --title="ROS 2 Perception" -- bash -c "
        echo '=========================================='
        echo ' ROS 2 DRONE PERCEPTION'
        echo '=========================================='
        echo ''
        echo 'Starting camera_viewer / drone_vision_node...'
        echo ''

        source /opt/ros/jazzy/setup.bash
        source '$ROS_WS/install/setup.bash'

        ros2 run drone_navigation_cpp camera_viewer

        EXIT_CODE=\$?

        echo ''
        echo 'Perception node terminated with exit code: \$EXIT_CODE'
        read -p 'Press ENTER to close this terminal...'
    "


    echo ""
    echo "============================================================"
    echo " Simulation startup sequence launched."
    echo "============================================================"
    echo ""
    echo "Running components:"
    echo "  1. PX4 + Gazebo"
    echo "  2. QGroundControl"
    echo "  3. MicroXRCEAgent"
    echo "  4. ROS 2 Camera Bridge"
    echo "  5. ROS 2 Perception"
    echo ""
    echo "Camera topic:"
    echo "  $CAMERA_TOPIC"
    echo ""
    echo "To stop everything:"
    echo "  ./start_simulation.sh stop"
    echo ""
    echo "To restart everything:"
    echo "  ./start_simulation.sh restart"
    echo ""
    echo "NOTE: drone_controller and offboard_path are NOT started automatically."
    echo ""
}


# ============================================================
# MAIN
# ============================================================

case "${1:-start}" in

    start)
        start_simulation
        ;;

    stop)
        stop_simulation
        ;;

    restart)
        stop_simulation
        sleep 2
        start_simulation
        ;;

    *)
        echo ""
        echo "Usage:"
        echo ""
        echo "  ./start_simulation.sh"
        echo "  ./start_simulation.sh stop"
        echo "  ./start_simulation.sh restart"
        echo ""
        exit 1
        ;;

esac