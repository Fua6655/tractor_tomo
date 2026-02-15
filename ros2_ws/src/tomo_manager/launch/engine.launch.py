from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="tomo_manager",
            executable="engine_manager",
            name="engine_manager",
            output="screen",
            parameters=[
                {
                    "states_topic": "/tomo/states",
                    "cmd_topic": "/tomo/cmd_vel",
                    "engine_cmd_topic": "/tomo/engine_cmd",
                    "idle_throttle": 0.0,
                    "start_throttle": 0.9,
                }
            ],
        )
    ])
