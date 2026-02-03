#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    turtlesim = Node(
        package="turtlesim",
        executable="turtlesim_node",
        name="turtlesim",
        output="screen",
        remappings=[
            ("/turtle1/cmd_vel", "/tomo/cmd_vel"),
        ],
    )

    return LaunchDescription([
        turtlesim,
    ])
