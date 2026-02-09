#!/usr/bin/env python3

import rclpy
import socket
import time
import threading

from rclpy.node import Node
from tomo_msgs.msg import OutputStates
from std_msgs.msg import Float32



class EspBridge(Node):

    def __init__(self):
        super().__init__("esp_bridge")

        # ---------------- PARAMETERS ----------------
        self.declare_parameter("output_topic", "/tomo/states")
        self.declare_parameter("engine_cmd_topic", "/tomo/engine_cmd")
        self.declare_parameter("steer_cmd_topic", "/tomo/steer_cmd")
        self.declare_parameter("esp_ip", "192.168.0.187")
        self.declare_parameter("esp_port", 8888)
        self.declare_parameter("heartbeat_rate", 0.5)
        self.declare_parameter("engine_watchdog_rate", 0.3)
        self.declare_parameter("engine_timeout", 1.0)
        self.declare_parameter("steer_watchdog_rate", 0.1)
        self.declare_parameter("steer_timeout", 0.5)

        self.output_topic = str(self.get_parameter('output_topic').value)
        self.engine_cmd_topic = str(self.get_parameter("engine_cmd_topic").value)
        self.steer_cmd_topic = str(self.get_parameter("steer_cmd_topic").value)
        self.esp_ip = self.get_parameter("esp_ip").value
        self.esp_port = self.get_parameter("esp_port").value
        self.heartbeat_rate = self.get_parameter("heartbeat_rate").value
        self.engine_watchdog_rate = self.get_parameter("engine_watchdog_rate").value
        self.engine_timeout = float(self.get_parameter("engine_timeout").value)
        self.steer_watchdog_rate = self.get_parameter("steer_watchdog_rate").value
        self.steer_timeout = self.get_parameter("steer_timeout").value

        # ---------------- UDP ----------------
        self.tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rx.bind(("", self.esp_port + 1))
        self.rx.settimeout(0.1)

        self.seq = 0
        self.pending = {}

        # ---------------- ROS ----------------
        self.last_engine_cmd_time = time.time()
        self.last_engine_value = 0.0
        self.last_steer_cmd_time = time.time()
        self.last_steer_value = 0.0

        self.create_subscription(OutputStates,self.output_topic,self.output_cb, 20)
        self.create_subscription(Float32, self.engine_cmd_topic, self.engine_cmd_cb, 20)
        self.create_subscription(Float32,self.steer_cmd_topic,self.steer_cmd_cb, 20)

        self.create_timer(self.heartbeat_rate, self.send_heartbeat)
        self.create_timer(self.engine_watchdog_rate, self.engine_watchdog)
        self.create_timer(self.steer_watchdog_rate, self.steer_watchdog)

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

    def engine_cmd_cb(self, msg: Float32):
        self.last_engine_cmd_time = time.time()
        self.last_engine_value = msg.data
        payload = self.serialize_engine(msg)
        self.send(payload)

    def steer_cmd_cb(self, msg: Float32):
        self.last_steer_cmd_time = time.time()
        self.last_steer_value = msg.data
        payload = self.serialize_steer(msg)
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

    @staticmethod
    def serialize_engine(msg: Float32) -> str:
        throttle = max(0.0, min(1.0, msg.data))  # clamp 0–1
        return f"THR,{throttle:.3f}"

    @staticmethod
    def serialize_steer(msg: Float32) -> str:
        steer = max(-1.0, min(1.0, msg.data))
        return f"STR,{steer:.3f}"

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

    # ==================================================
    # ENGINE WATCHDOG
    # ==================================================
    def engine_watchdog(self):
        if time.time() - self.last_engine_cmd_time > self.engine_timeout:
            if self.last_engine_value != 0.0:
                self.last_engine_value = 0.0
                payload = "THR,0.00"
                self.send(payload)
                self.get_logger().warn("⚠️ Engine watchdog timeout → throttle=0")

    def steer_watchdog(self):
        if time.time() - self.last_steer_cmd_time > self.steer_timeout:
            if self.last_steer_value != 0.0:
                self.last_steer_value = 0.0
                self.send("STR,0.000")
                self.get_logger().warn("⚠️ Steering watchdog timeout → steer=0")


def main():
    rclpy.init()
    node = EspBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
