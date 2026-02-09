#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from tomo_msgs.msg import OutputStates


class EngineManager(Node):

    def __init__(self):
        super().__init__("engine_manager")

        # PARAMETERS
        self.declare_parameter("states_topic", "/tomo/states")
        self.declare_parameter("cmd_topic", "/tomo/cmd_vel")
        self.declare_parameter("engine_cmd_topic", "/tomo/engine_cmd")

        self.declare_parameter("idle_throttle", 0.15)
        self.declare_parameter("start_throttle", 0.9)

        self.states_topic = self.get_parameter("states_topic").value
        self.cmd_topic = self.get_parameter("cmd_topic").value
        self.engine_cmd_topic = self.get_parameter("engine_cmd_topic").value

        self.idle = float(self.get_parameter("idle_throttle").value)
        self.start = float(self.get_parameter("start_throttle").value)

        # STATE
        self.last_states = OutputStates()
        self.last_cmd = Twist()

        # ROS
        self.create_subscription(OutputStates, self.states_topic, self.states_cb, 20)
        self.create_subscription(Twist, self.cmd_topic, self.cmd_cb, 20)

        self.pub = self.create_publisher(Float32, self.engine_cmd_topic, 20)

        self.get_logger().info("EngineManager READY")

    def states_cb(self, msg: OutputStates):
        self.last_states = msg
        self.compute()

    def cmd_cb(self, msg: Twist):
        self.last_cmd = msg
        self.compute()

    def compute(self):
        s = self.last_states
        cmd = max(0.0, min(1.0, self.last_cmd.linear.x))

        # ENGINE STOP
        if s.engine_stop:
            throttle = 0.0

        # ENGINE START HELD
        elif s.engine_start:
            throttle = self.start

        # MOVE ALLOWED → follow cmd_vel
        elif s.move_allowed:
            throttle = self.idle + cmd * (self.start - self.idle)

        # IDLE
        else:
            throttle = self.idle

        msg = Float32()
        msg.data = float(throttle)
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = EngineManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
