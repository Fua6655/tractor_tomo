from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        Node(
            package='tomo_web',
            executable='web_node',
            name='tomo_web',
            output='screen',
            parameters=[
                {
                    'control_event_topic': '/control/events',
                    'control_emergency_topic': '/control/emergency',
                    'output_topic': '/tomo/states',
                    'esp_ip': '255.255.255.255',
                }
            ]
        )

    ])
