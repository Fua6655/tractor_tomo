#!/usr/bin/env python3

import rclpy
import socket
import time
import threading

from rclpy.node import Node
from tomo_msgs.msg import OutputStates


class EspBridge(Node):

    def __init__(self):
        super().__init__("esp_bridge")

        # ---------------- PARAMETERS ----------------
        self.declare_parameter("output_topic", "/tomo/states")
        self.declare_parameter("esp_ip", "192.168.0.187")
        self.declare_parameter("esp_port", 8888)
        self.declare_parameter("heartbeat_rate", 0.5)

        self.output_topic = str(self.get_parameter('output_topic').value)
        self.esp_ip = self.get_parameter("esp_ip").value
        self.esp_port = self.get_parameter("esp_port").value
        self.heartbeat_rate = self.get_parameter("heartbeat_rate").value

        # ---------------- UDP ----------------
        self.tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rx.bind(("", self.esp_port + 1))
        self.rx.settimeout(0.1)

        self.seq = 0
        self.pending = {}

        # ---------------- ROS ----------------
        self.create_subscription(OutputStates,self.output_topic,self.output_cb,20)

        self.create_timer(self.heartbeat_rate, self.send_heartbeat)

        threading.Thread(target=self.rx_loop, daemon=True).start()

        self.get_logger().info(
            f"ESP bridge → {self.esp_ip}:{self.esp_port}"
        )

    # ==================================================
    # OUTPUT CALLBACK
    # ==================================================
    def output_cb(self, msg: OutputStates):
        payload = self.serialize(msg)
        self.send(payload)

    # ==================================================
    # SERIALIZATION
    # ==================================================
    @staticmethod
    def serialize(s: OutputStates) -> str:
        return (
            "OUT,"
            f"{int(s.armed_state)},"
            f"{int(s.power_state)},"
            f"{int(s.light_state)},"
            f"{int(s.engine_start)},"
            f"{int(s.engine_stop)},"
            f"{int(s.clutch_active)},"
            f"{int(s.brake_active)},"
            f"{int(s.move_allowed)},"
            f"{int(s.front_position)},"
            f"{int(s.front_short)},"
            f"{int(s.front_long)},"
            f"{int(s.back_position)},"
            f"{int(s.left_blink)},"
            f"{int(s.right_blink)},"
        )

    # ==================================================
    # UDP SEND
    # ==================================================
    def send(self, payload: str):
        self.seq += 1
        msg = f"CMD,{self.seq},{payload}"
        self.pending[self.seq] = time.time()
        self.tx.sendto(msg.encode(), (self.esp_ip, self.esp_port))

    # ==================================================
    # HEARTBEAT
    # ==================================================
    def send_heartbeat(self):
        self.seq += 1
        self.pending[self.seq] = time.time()
        self.tx.sendto(
            f"HEARTBEAT,{self.seq}".encode(),
            (self.esp_ip, self.esp_port)
        )

    # ==================================================
    # RX LOOP (ACK + latency)
    # ==================================================
    def rx_loop(self):
        while rclpy.ok():
            try:
                data, _ = self.rx.recvfrom(256)
            except:
                continue

            msg = data.decode().strip()

            if msg.startswith("ACK"):
                try:
                    seq = int(msg.split(",")[1])
                except:
                    continue

                if seq in self.pending:
                    latency = (time.time() - self.pending.pop(seq)) * 1000
                    self.get_logger().debug(
                        f"ESP latency: {latency:.1f} ms"
                    )


def main():
    rclpy.init()
    node = EspBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
