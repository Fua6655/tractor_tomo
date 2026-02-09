from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tomo_manager',
            executable='steering_manager',
            name='steering_manager',
            output='screen',
        )
    ])
