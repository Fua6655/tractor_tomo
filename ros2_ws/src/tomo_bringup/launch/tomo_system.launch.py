#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("tomo_factory"),
                "launch",
                "control.launch.py",
            )
        )
    )

    motion_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("tomo_factory"),
                "launch",
                "motion.launch.py",
            )
        )
    )

    ps4_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("tomo_ps4"),
                "launch",
                "ps4.launch.py",
            )
        )
    )

    esp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("tomo_esp"),
                "launch",
                "esp.launch.py",
            )
        )
    )

    web_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("tomo_web"),
                "launch",
                "web.launch.py",
            )
        )
    )

    return LaunchDescription([
        control_launch,
        motion_launch,
        ps4_launch,
        esp_launch,
        web_launch,   # ← ako želiš web
    ])
