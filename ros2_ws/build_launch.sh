#!/usr/bin/env bash

echo "📂 Going to ROS2 workspace"
cd ~/tractor_tomo/ros2_ws

echo "🧹 Cleaning build folders"
rm -rf build/ install/ log/

echo "🌍 Sourcing ROS Jazzy"
source /opt/ros/jazzy/setup.bash

echo "🔨 Building workspace"
colcon build
		# --symlink-install
		# --packages-select tomo_auto tomo_bringup tomo_esp tomo_factory tomo_msgs tomo_ps4 tomo_web
		# --event-handlers console_direct+

echo "🌍 Sourcing workspace"
source install/setup.bash

echo "🚜 Launching TOMO"
ros2 launch tomo_bringup tomo_system.launch.py

