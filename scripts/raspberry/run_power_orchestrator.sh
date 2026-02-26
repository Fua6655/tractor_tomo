#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="${TOMO_REPO_DIR:-$HOME/tractor_tomo}"

cd "$REPO_DIR"

source /opt/ros/jazzy/setup.bash
source "$REPO_DIR/ros2_ws/install/setup.bash"

exec python3 "$REPO_DIR/scripts/raspberry/power_orchestrator.py"
