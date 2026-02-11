#!/usr/bin/env bash

echo "📂 Going to ROS2 workspace"
cd ~/tractor_tomo/ros2_ws

echo "🌍 Sourcing workspace"
source install/setup.bash

echo "🚜 Launching TOMO"
ros2 launch tomo_bringup tomo_system.launch.py