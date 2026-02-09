#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from tomo_msgs.msg import Emergency, OutputStates


class MotionFactory(Node):
    """
    MotionFactory
    - Mux PS4 / AUTO cmd_vel
    - Emergency → instant zero cmd_vel
    - Optional failsafe bypass
    """

    def __init__(self):
        super().__init__("motion_factory")

        # ---------------- PARAMETERS ----------------
        self.declare_parameter("ps4_cmd_topic", "/ps4/cmd_vel")
        self.declare_parameter("auto_cmd_topic", "/auto/cmd_vel")
        self.declare_parameter("output_cmd_topic", "/tomo/cmd_vel")
        self.declare_parameter("states_topic", "/tomo/states")
        self.declare_parameter("control_emergency_topic", "/control/emergency")
        self.declare_parameter("failsafe_enabled", True)
        self.declare_parameter("max_lin_accel", 0.5)  # m/s^2
        self.declare_parameter("max_ang_accel", 1.5)  # rad/s^2
        self.declare_parameter("control_rate", 20.0)  # Hz
        self.declare_parameter("max_lin_jerk", 1.0)  # m/s^3
        self.declare_parameter("max_ang_jerk", 3.0)  # rad/s^3

        self.ps4_cmd_topic = self.get_parameter("ps4_cmd_topic").value
        self.auto_cmd_topic = self.get_parameter("auto_cmd_topic").value
        self.output_cmd_topic = self.get_parameter("output_cmd_topic").value
        self.states_topic = self.get_parameter("states_topic").value
        self.emergency_topic = self.get_parameter("control_emergency_topic").value
        self.failsafe_enabled = bool(self.get_parameter("failsafe_enabled").value)
        self.max_lin_accel = float(self.get_parameter("max_lin_accel").value)
        self.max_ang_accel = float(self.get_parameter("max_ang_accel").value)
        self.control_rate = float(self.get_parameter("control_rate").value)
        self.dt = 1.0 / self.control_rate
        self.max_lin_jerk = float(self.get_parameter("max_lin_jerk").value)
        self.max_ang_jerk = float(self.get_parameter("max_ang_jerk").value)

        # ---------------- STATE ----------------
        self.active_source = "PS4"   # PS4 | AUTO
        self.emergency_active = False
        self.emergency_level = None

        self.soft_emergency_active = False
        self.ramp_twist = Twist()
        self.ramp_step = 0.1

        self.prev_source = self.active_source
        self.source_switching = False
        self.switch_ramp_eps = 0.02

        self.prev_accel_lin = 0.0
        self.prev_accel_ang = 0.0

        self.prev_cmd = Twist()
        self.last_ps4_cmd = Twist()
        self.last_auto_cmd = Twist()

        self.engine_stop = False
        self.clutch_active = False
        self.brake_active = False
        self.move_allowed = True

        # ---------------- ROS ----------------
        self.pub_cmd = self.create_publisher(Twist, self.output_cmd_topic, 30)
        self.create_subscription(Twist, self.ps4_cmd_topic, self.ps4_cmd_cb, 30)
        self.create_subscription(Twist, self.auto_cmd_topic, self.auto_cmd_cb, 30)
        self.create_subscription(Emergency, self.emergency_topic, self.emergency_cb, 10)
        self.create_subscription(OutputStates, self.states_topic, self.states_cb, 20)

        self.timer = self.create_timer(0.05, self.timer_cb)

        self.get_logger().info(
            f"🚜 MotionFactory READY | failsafe_enabled={self.failsafe_enabled}"
        )

    # ==================================================
    # CALLBACKS
    # ==================================================

    def ps4_cmd_cb(self, msg: Twist):
        self.last_ps4_cmd = msg
        if self.active_source == "PS4":
            self.publish_muxed()

    def auto_cmd_cb(self, msg: Twist):
        self.last_auto_cmd = msg
        if self.active_source == "AUTO":
            self.publish_muxed()

    def emergency_cb(self, msg: Emergency):
        if not self.failsafe_enabled:
            return

        self.emergency_active = msg.active
        self.emergency_level = msg.level

        if msg.active and msg.level == Emergency.LEVEL_HARD:
            self.publish_zero()

    def timer_cb(self):
        if not self.emergency_active:
            return

        if self.emergency_level == Emergency.LEVEL_SOFT:
            self.publish_soft_ramp()

    def states_cb(self, msg: OutputStates):
        self.engine_stop = msg.engine_stop
        self.clutch_active = msg.clutch_active
        self.brake_active = msg.brake_active
        self.move_allowed = msg.move_allowed

        new_source = "AUTO" if msg.source == OutputStates.SOURCE_AUTO else "PS4"
        if new_source != self.active_source:
            self.prev_source = self.active_source
            self.active_source = new_source
            self.source_switching = True

    # ==================================================
    # CORE LOGIC
    # ==================================================

    def publish_muxed(self):
        # =========================
        # SOURCE SWITCH RAMP
        # =========================
        if self.source_switching:
            zero = self.zero_twist()
            smoothed = self.smooth_cmd(zero)
            self.pub_cmd.publish(smoothed)

            if (
                    abs(smoothed.linear.x) < self.switch_ramp_eps and
                    abs(smoothed.angular.z) < self.switch_ramp_eps
            ):
                self.source_switching = False
                self.prev_cmd = zero
                self.prev_accel_lin = 0.0
                self.prev_accel_ang = 0.0

            return

        if not self.move_allowed:
            zero = Twist()
            smoothed = self.smooth_cmd(zero)
            self.pub_cmd.publish(smoothed)
            return

        # =========================
        # ACTIVE SOURCE
        # =========================
        if self.active_source == "AUTO":
            src = self.last_auto_cmd
        else:
            src = self.last_ps4_cmd

        target = self.sanitize_twist(src)
        smoothed = self.smooth_cmd(target)

        self.ramp_twist = smoothed
        self.pub_cmd.publish(smoothed)

    def publish_zero(self):
        zero = self.zero_twist()
        self.prev_cmd = zero
        self.prev_accel_lin = 0.0
        self.prev_accel_ang = 0.0
        self.pub_cmd.publish(zero)

    def publish_soft_ramp(self):
        src = self.last_ps4_cmd if self.active_source == "PS4" else self.last_auto_cmd

        self.ramp_twist.linear.x *= (1.0 - self.ramp_step)
        self.ramp_twist.angular.z *= (1.0 - self.ramp_step)

        if abs(self.ramp_twist.linear.x) < 0.01:
            self.publish_zero()
            return

        self.pub_cmd.publish(self.sanitize_twist(self.ramp_twist))

    def smooth_cmd(self, target: Twist) -> Twist:
        out = Twist()

        desired_accel_lin = (target.linear.x - self.prev_cmd.linear.x) / self.dt
        desired_accel_ang = (target.angular.z - self.prev_cmd.angular.z) / self.dt

        accel_lin = self.limit_jerk(
            desired_accel_lin,
            self.prev_accel_lin,
            self.max_lin_jerk
        )

        accel_ang = self.limit_jerk(
            desired_accel_ang,
            self.prev_accel_ang,
            self.max_ang_jerk
        )

        accel_lin = self.limit_rate(
            accel_lin,
            self.prev_accel_lin,
            self.max_lin_accel
        )

        accel_ang = self.limit_rate(
            accel_ang,
            self.prev_accel_ang,
            self.max_ang_accel
        )

        out.linear.x = self.prev_cmd.linear.x + accel_lin * self.dt
        out.angular.z = self.prev_cmd.angular.z + accel_ang * self.dt

        out.linear.y = out.linear.z = 0.0
        out.angular.x = out.angular.y = 0.0

        self.prev_cmd = out
        self.prev_accel_lin = accel_lin
        self.prev_accel_ang = accel_ang

        return out

    # ==================================================
    # HELPERS
    # ==================================================

    @staticmethod
    def zero_twist() -> Twist:
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        return msg

    @staticmethod
    def sanitize_twist(src: Twist) -> Twist:
        msg = Twist()

        msg.linear.x = float(src.linear.x)
        msg.linear.y = 0.0
        msg.linear.z = 0.0

        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = float(src.angular.z)

        return msg

    def limit_rate(self, target: float, current: float, max_delta: float) -> float:
        delta = target - current
        if delta > max_delta:
            return current + max_delta
        if delta < -max_delta:
            return current - max_delta
        return target

    def limit_jerk(self, target_accel, prev_accel, max_jerk):
        max_delta = max_jerk * self.dt
        delta = target_accel - prev_accel

        if delta > max_delta:
            return prev_accel + max_delta
        if delta < -max_delta:
            return prev_accel - max_delta
        return target_accel


def main():
    rclpy.init()
    node = MotionFactory()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
