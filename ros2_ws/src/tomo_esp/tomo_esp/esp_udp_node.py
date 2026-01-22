#!/usr/bin/env python3

import rclpy
import socket
from rclpy.node import Node
from tomo_msgs.msg import OutputStates


class EspUdpNode(Node):

    def __init__(self):
        super().__init__("tomo_esp")

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.create_subscription(
            OutputStates,
            "tomo/output",
            self.cb,
            10,
        )

        self.get_logger().info("ESP UDP bridge ready")

    def cb(self, msg: OutputStates):
        ip = msg.esp_ip
        port = msg.esp_port

        payload = (
            f"STATES,"
            f"{int(msg.armed)},"
            f"{int(msg.power_mode)},"
            f"{int(msg.light_mode)}|"
            f"EVENTS,"
            f"{int(msg.engine_start)},"
            f"{int(msg.clutch_down)},"
            f"{int(msg.high_speed)},"
            f"{int(msg.move_allowed)}|"
            f"LIGHTS,"
            f"{int(msg.front_position)},"
            f"{int(msg.front_short)},"
            f"{int(msg.front_long)},"
            f"{int(msg.back_light)},"
            f"{int(msg.left_blink)},"
            f"{int(msg.right_blink)}"
        )

        self.sock.sendto(payload.encode(), (ip, port))


def main():
    rclpy.init()
    rclpy.spin(EspUdpNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
