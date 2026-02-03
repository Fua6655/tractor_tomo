#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        Node(
            package="tomo_factory",
            executable="motion_factory",
            name="motion_factory",
            output="screen",
            parameters=[{
                # --- topics ---
                "ps4_cmd_topic": "/ps4/cmd_vel",
                "auto_cmd_topic": "/auto/cmd_vel",
                "output_cmd_topic": "/tomo/cmd_vel",
                "states_topic": "/tomo/states",
                "control_emergency_topic": "/control/emergency",

                # --- safety ---
                "failsafe_enabled": True,

                # --- motion limits ---
                "control_rate": 20.0,      # Hz
                "max_lin_accel": 0.5,      # m/s^2
                "max_ang_accel": 1.5,      # rad/s^2
                "max_lin_jerk": 1.0,       # m/s^3
                "max_ang_jerk": 3.0,       # rad/s^3
            }]
        )

    ])
