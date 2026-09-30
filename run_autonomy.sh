#!/usr/bin/env bash

set -e

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

CONFIG="$PROJECT_DIR/config/autonomy.env"
START_PERCEPTION="$PROJECT_DIR/scripts/start_perception.sh"
START_FLIGHT="$PROJECT_DIR/scripts/start_flight.sh"
STOP_ALL="$PROJECT_DIR/scripts/stop_all.sh"
CHECK_SYSTEM="$PROJECT_DIR/scripts/check_system.sh"
SIMULATION="$PROJECT_DIR/start_simulation.sh"

source "$CONFIG"

usage()
{
    echo
    echo "Autonomous Drone Navigation"
    echo
    echo "Usage:"
    echo
    echo "  ./run_autonomy.sh"
    echo "      Start simulation + perception"
    echo
    echo "  ./run_autonomy.sh flight"
    echo "      Start OFFBOARD avoidance controller"
    echo
    echo "  ./run_autonomy.sh demo"
    echo "      Start complete autonomous system"
    echo
    echo "  ./run_autonomy.sh check"
    echo "      Check system status"
    echo
    echo "  ./run_autonomy.sh stop"
    echo "      Stop all project processes"
    echo
    echo "  ./run_autonomy.sh restart"
    echo "      Restart simulation + perception"
    echo
}

start_base()
{
    echo
    echo "=========================================="
    echo " Starting Autonomous Drone Navigation"
    echo "=========================================="

    "$SIMULATION"

    echo
    echo "Waiting for simulation startup..."
    sleep 8

    "$START_PERCEPTION"

    echo
    echo "=========================================="
    echo " Perception pipeline READY"
    echo "=========================================="
}

case "${1:-start}" in

    start)
        start_base
        ;;

    flight)
        "$START_FLIGHT"
        ;;

    demo)
        start_base

        echo
        echo "Waiting for perception pipeline..."
        sleep 5

        "$START_FLIGHT"

        echo
        echo "=========================================="
        echo " AUTONOMOUS DEMO STARTED"
        echo "=========================================="
        ;;

    check)
        "$CHECK_SYSTEM"
        ;;

    stop)
        "$STOP_ALL"
        ;;

    restart)
        "$STOP_ALL"

        echo
        echo "Waiting before restart..."
        sleep 2

        start_base
        ;;

    *)
        usage
        ;;

esac