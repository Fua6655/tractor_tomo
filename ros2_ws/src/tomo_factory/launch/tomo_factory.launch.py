from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ---------------- LAUNCH ARGUMENTS ----------------
    failsafe_enabled = LaunchConfiguration("failsafe_enabled")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([

        DeclareLaunchArgument(
            "failsafe_enabled",
            default_value="true",
            description="Enable ESP / motion failsafe"
        ),

        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation time"
        ),

        # ---------------- MOTION FACTORY ----------------
        Node(
            package="tomo_factory",
            executable="motion_factory",
            name="motion_factory",
            output="screen",
            parameters=[{
                "failsafe_enabled": failsafe_enabled,
                "use_sim_time": use_sim_time,
                "ps4_cmd_topic": "/ps4/cmd_vel",
                "auto_cmd_topic": "/auto/cmd_vel",
                "output_cmd_topic": "/tomo/cmd_vel",
                "states_topic": "/tomo/states",
            }],
        ),

        # ---------------- CONTROL FACTORY ----------------
        Node(
            package="tomo_factory",
            executable="control_factory",
            name="tomo_factory",
            output="screen",
            parameters=[{
                "control_event_topic": "/control/events",
                "emergency_topic": "/control/emergency",
                "ps4_cmd_vel_topic": "/ps4/cmd_vel",
                "auto_cmd_vel_topic": "/auto/cmd_vel",
                "output_topic": "/tomo/states",
                "output_cmd_topic": "/tomo/cmd_vel",
                "ps4_timeout": 0.5,
                "web_timeout": 1.0,
                "auto_timeout": 0.5,
            }]
        ),
    ])
