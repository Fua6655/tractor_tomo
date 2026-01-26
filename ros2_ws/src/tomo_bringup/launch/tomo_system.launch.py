from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    # ==================================================
    # LAUNCH ARGUMENTS
    # ==================================================
    esp_ip   = LaunchConfiguration("esp_ip")
    esp_port = LaunchConfiguration("esp_port")

    joy_dev  = LaunchConfiguration("joy_dev")

    return LaunchDescription([

        # ================= ARGUMENTS =================
        DeclareLaunchArgument(
            "esp_ip",
            default_value="192.168.0.187",
            description="ESP32 IP address"
        ),

        DeclareLaunchArgument(
            "esp_port",
            default_value="8888",
            description="ESP32 UDP port"
        ),

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
                "linear_axis": 1,
                "angular_axis": 0,
                "arm_hold_time": 2.0,
                "deadzone": 0.08,
                "linear_scale": 0.4,
                "angular_scale": 1.0,
                "joy_topic": "/joy",
                "control_event_topic": "/control/events",
                "control_emergency_topic": "/control/emergency",
                "cmd_topic": "/ps4/cmd_vel",



            }]
        ),

        # ================= WEB → ControlEvents =================
        Node(
            package="tomo_web",
            executable="web_node",
            name="tomo_web",
            output="screen",
            parameters=[{
                "control_event_topic": "/control/events",
                "emergency_topic": "/control/emergency",
                "output_topic": "/tomo/states",
            }]
        ),

        # ================= FACTORY (REDUCER + STATE MACHINE) =================
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

        # ================= ESP BRIDGE =================
        Node(
            package="tomo_esp",
            executable="esp_bridge",
            name="tomo_esp",
            output="screen",
            parameters=[{
                "output_topic": "/tomo/states",
                "esp_ip": esp_ip,
                "esp_port": esp_port,
                "heartbeat_hz": 5.0,
            }]
        ),
    ])
