#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from tomo_msgs.msg import Emergency


class SteeringManager(Node):

    def __init__(self):
        super().__init__("steering_manager")

        # ---------------- PARAMETERS ----------------
        self.declare_parameter("cmd_vel_topic", "/tomo/cmd_vel")
        self.declare_parameter("steer_cmd_topic", "/tomo/steer_cmd")
        self.declare_parameter("control_emergency_topic", "/control/emergency")

        self.declare_parameter("max_angle_deg", 35.0)
        self.declare_parameter("soft_limit_margin_deg", 5.0)
        self.declare_parameter("deadzone", 0.05)
        self.declare_parameter("steer_scale", 1.0)
        self.declare_parameter("control_rate", 50.0)
        self.declare_parameter("cmd_timeout", 0.3)
        self.declare_parameter("max_steer_rate", 1.0)

        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.steer_cmd_topic = self.get_parameter("steer_cmd_topic").value
        self.emergency_topic = self.get_parameter("control_emergency_topic").value

        self.max_angle = self.get_parameter("max_angle_deg").value
        self.soft_margin = self.get_parameter("soft_limit_margin_deg").value
        self.deadzone = self.get_parameter("deadzone").value
        self.steer_scale = self.get_parameter("steer_scale").value
        self.rate = self.get_parameter("control_rate").value
        self.cmd_timeout = self.get_parameter("cmd_timeout").value
        self.max_steer_rate = self.get_parameter("max_steer_rate").value


        self.dt = 1.0 / self.rate
        self.last_cmd_time = self.get_clock().now()
        self.last_published = 0.0

        # ---------------- STATE ----------------
        self.theta_est = 0.0  # degrees
        self.emergency_active = False
        self.last_cmd = 0.0

        # ---------------- ROS ----------------
        self.pub = self.create_publisher(Float32, self.steer_cmd_topic, 20)

        self.create_subscription(Twist, self.cmd_vel_topic, self.cmd_vel_cb, 20)
        self.create_subscription(Emergency, self.emergency_topic, self.emergency_cb, 10)

        self.timer = self.create_timer(self.dt, self.update)

        self.get_logger().info("🦾 SteeringManager READY")

    # ==================================================
    # CALLBACKS
    # ==================================================

    def cmd_vel_cb(self, msg: Twist):
        self.last_cmd = msg.angular.z
        self.last_cmd_time = self.get_clock().now()

    def emergency_cb(self, msg: Emergency):
        self.emergency_active = msg.active
        if self.emergency_active:
            self.theta_est = 0.0
            self.last_published = 0.0
            self.publish(0.0)
            return

    # ==================================================
    # CORE LOOP
    # ==================================================

    def update(self):
        now = self.get_clock().now()
        time_since_cmd = (now - self.last_cmd_time).nanoseconds * 1e-9
        if time_since_cmd > self.cmd_timeout:
            self.last_cmd = 0.0

        if self.emergency_active:
            self.publish(0.0)
            return

        cmd = self.apply_deadzone(self.last_cmd)
        cmd *= self.steer_scale


        max_delta = self.max_steer_rate * self.dt
        cmd = max(
            self.last_published - max_delta,
            min(self.last_published + max_delta, cmd)
        )
        # integrate virtual angle
        self.theta_est += cmd * self.dt * 180.0  # deg/s approx

        # clamp hard limits
        self.theta_est = max(
            -self.max_angle,
            min(self.max_angle, self.theta_est)
        )

        cmd = self.apply_soft_limits(cmd)

        self.publish(cmd)
        self.last_published = cmd

    # ==================================================
    # HELPERS
    # ==================================================

    def apply_deadzone(self, v):
        if abs(v) < self.deadzone:
            return 0.0
        return v

    def apply_soft_limits(self, cmd):
        if self.theta_est > self.max_angle - self.soft_margin:
            return min(cmd, 0.0)
        if self.theta_est < -self.max_angle + self.soft_margin:
            return max(cmd, 0.0)
        return cmd

    def publish(self, value):
        msg = Float32()
        msg.data = max(-1.0, min(1.0, value))
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = SteeringManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
