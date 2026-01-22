from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        Node(
            package="tomo_factory",
            executable="control_factory",
            name="control_factory",
            parameters=[{
                "esp_ip": "192.168.0.187",
                "esp_port": 8888,
                "gpio_map": [2,3,4,5,6,7,8,9,10]
            }],
            output="screen"
        ),

        Node(
            package="tomo_esp",
            executable="esp_udp_node",
            name="tomo_esp",
            output="screen"
        ),
        Node(
            package="tomo_ps4",
            executable="ps4_node",
            name="tomo_ps4",
            parameters=[{
                "joy_topic": "/joy",
                "linear_axis": 1,
                "angular_axis": 0,
                "deadzone": 0.08,
                "linear_scale": 1.0,
                "angular_scale": 2.0
            }],
            output="screen"
        ),
        Node(
            package="tomo_web",
            executable="web_server",
            name="tomo_web",
            output="screen"
        ),
    ])
