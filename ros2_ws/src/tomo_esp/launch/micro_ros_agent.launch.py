#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    port = LaunchConfiguration("port")

    return LaunchDescription([
        DeclareLaunchArgument(
            "port",
            default_value="8888",
            description="micro-ROS agent UDP port",
        ),
        ExecuteProcess(
            cmd=[
                "bash",
                os.path.join(
                    get_package_share_directory("tomo_esp"),
                    "scripts",
                    "start_micro_ros_agent.sh",
                ),
                port,
            ],
            output="screen",
        ),
    ])
