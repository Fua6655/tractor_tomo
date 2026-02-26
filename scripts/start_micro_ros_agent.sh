#!/usr/bin/env bash

echo "📂 Going to ROS2 workspace"
cd ~/tractor_tomo/ros2_ws

echo "🌍 Sourcing workspace"
source install/setup.bash

#echo "🛑 Stopping existing micro-ROS agent on port 8888 (if any)"
#if command -v sudo >/dev/null 2>&1; then
#  sudo ss -lunp | awk '/:8888 / { for (i=1; i<=NF; i++) if ($i ~ /pid=/) print $i }' \
#    | sed -E 's/.*pid=([0-9]+).*/\1/' \
#    | sort -u \
#    | xargs -r sudo kill
#fi

echo "🚜 Launching micro-ros agent"
ros2 launch tomo_esp micro_ros_agent.launch.py