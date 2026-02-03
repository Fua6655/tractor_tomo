#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        Node(
            package="tomo_esp",
            executable="esp_bridge",
            name="esp_bridge",
            output="screen",
            parameters=[{
                "output_topic": "/tomo/states",
                "esp_ip": "192.168.0.187",
                "esp_port": 8888,
                "heartbeat_rate": 0.5,
            }]
        )

    ])
