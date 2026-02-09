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
                "engine_cmd_topic": "/tomo/engine_cmd",
                "esp_ip": "192.168.0.187",
                "esp_port": 8888,
                "heartbeat_rate": 0.5,
                "engine_watchdog_rate": 0.3,
                "engine_timeout": 1.0,
            }]
        )

    ])
