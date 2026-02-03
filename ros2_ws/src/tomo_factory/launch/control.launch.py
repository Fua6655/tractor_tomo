from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        Node(
            package='tomo_factory',
            executable='control_factory',
            name='control_factory',
            output='screen',
            parameters=[{
                'control_event_topic': '/control/events',
                'control_emergency_topic': '/control/emergency',
                'output_topic': '/tomo/states',
            }]
        )

    ])
