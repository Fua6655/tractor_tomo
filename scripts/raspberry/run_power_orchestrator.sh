#!/usr/bin/env bash
set -eo pipefail

REPO_DIR="${TOMO_REPO_DIR:-$HOME/tractor_tomo}"

cd "$REPO_DIR"

# Some ROS setup scripts reference AMENT_TRACE_SETUP_FILES even when unset.
export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES-}"
source /opt/ros/jazzy/setup.bash
source "$REPO_DIR/ros2_ws/install/setup.bash"

exec python3 "$REPO_DIR/scripts/raspberry/power_orchestrator.py"
