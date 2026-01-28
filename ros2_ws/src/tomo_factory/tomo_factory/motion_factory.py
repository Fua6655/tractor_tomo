#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from tomo_msgs.msg import Emergency, OutputStates


class MotionFactory(Node):
    def __init__(self):
        super().__init__("motion_factory")

        # ---------------- PARAMETERS ----------------
        self.declare_parameter("ps4_cmd_topic", "/ps4/cmd_vel")
        self.declare_parameter("auto_cmd_topic", "/auto/cmd_vel")
        self.declare_parameter("out_cmd_topic", "/tomo/cmd_vel")
        self.declare_parameter("control_emergency_topic", "/control/emergency")
        self.declare_parameter("failsafe_enabled", True)

        self.ps4_cmd_topic = self.get_parameter("ps4_cmd_topic").value
        self.auto_cmd_topic = self.get_parameter("auto_cmd_topic").value
        self.out_cmd_topic = self.get_parameter("out_cmd_topic").value
        self.emergency_topic = self.get_parameter("control_emergency_topic").value
        self.failsafe_enabled = self.get_parameter("failsafe_enabled").value

        # ---------------- STATE ----------------
        self.active_source = "PS4"
        self.hard_emergency = False

        self.last_ps4_cmd = Twist()
        self.last_auto_cmd = Twist()

        # ---------------- ROS ----------------
        self.pub_cmd = self.create_publisher(
            Twist, self.out_cmd_topic, 30
        )

        self.create_subscription(
            Twist, self.ps4_cmd_topic, self.ps4_cmd_cb, 30
        )

        self.create_subscription(
            Twist, self.auto_cmd_topic, self.auto_cmd_cb, 30
        )

        self.create_subscription(
            Emergency, self.emergency_topic, self.emergency_cb, 10
        )

        self.get_logger().info("🧠 Motion factory READY")

    # ==================================================
    # CALLBACKS
    # ==================================================

    def ps4_cmd_cb(self, msg: Twist):
        self.last_ps4_cmd = msg
        self.publish_cmd()

    def auto_cmd_cb(self, msg: Twist):
        self.last_auto_cmd = msg
        self.publish_cmd()

    def emergency_cb(self, msg: Emergency):
        self.hard_emergency = msg.active and msg.level == Emergency.LEVEL_HARD
        self.publish_cmd()

    # ==================================================
    # CORE LOGIC
    # ==================================================

    def publish_cmd(self):
        out = Twist()

        # ---------- HARD EMERGENCY ----------
        if self.hard_emergency:
            self.pub_cmd.publish(out)
            return

        # ---------- SOURCE SELECT ----------
        if self.active_source == "PS4":
            src = self.last_ps4_cmd
        else:
            src = self.last_auto_cmd

        # ---------- COPY FULL TWIST ----------
        out.linear.x = src.linear.x
        out.linear.y = src.linear.y
        out.linear.z = src.linear.z
        out.angular.x = src.angular.x
        out.angular.y = src.angular.y
        out.angular.z = src.angular.z

        self.pub_cmd.publish(out)


def main():
    rclpy.init()
    node = MotionFactory()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
