from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    # ==================================================
    # LAUNCH ARGUMENTS
    # ==================================================

    joy_dev  = LaunchConfiguration("joy_dev")

    return LaunchDescription([

        # ================= ARGUMENTS =================
        DeclareLaunchArgument(
            "joy_dev",
            default_value="/dev/input/js0",
            description="Joystick device"
        ),

        # ================= JOYSTICK DRIVER =================
        Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
            output="screen",
            parameters=[{
                "dev": joy_dev,
                "deadzone": 0.05,
                "autorepeat_rate": 20.0
            }]
        ),

        # ================= PS4 → ControlEvents =================
        Node(
            package="tomo_ps4",
            executable="ps4_node",
            name="tomo_ps4",
            output="screen",
            parameters=[{
                "arm_hold_time": 2.0,
                "power_hold_time": 1.0,
                "light_hold_time": 1.0,
                "move_hold_time": 2.0,
                "joy_topic": "/joy",
                "control_event_topic": "/control/events",
                "control_emergency_topic": "/control/emergency",
                "cmd_topic": "/ps4/cmd_vel",
            }]
        ),
    ])
