#!/usr/bin/env python3

import time
from typing import List

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

from tomo_msgs.msg import ControlEvents, Emergency
from .ps4_controller import PS4Controller


class PS4Node(Node):

    def __init__(self):
        super().__init__("ps4_node")

        # ---------------- PARAMETERS ----------------
        self.declare_parameter("arm_hold_time", 2.0)
        self.declare_parameter("power_hold_time", 1.0)
        self.declare_parameter("light_hold_time", 1.0)
        self.declare_parameter("move_hold_time", 2.0)
        self.declare_parameter("joy_topic", "/joy")
        self.declare_parameter("control_event_topic", "/control/events")
        self.declare_parameter("control_emergency_topic", "/control/emergency")
        self.declare_parameter('cmd_topic', '/ps4/cmd_vel')

        self.arm_hold = self.get_parameter("arm_hold_time").value
        self.power_hold = self.get_parameter("power_hold_time").value
        self.light_hold = self.get_parameter("light_hold_time").value
        self.move_hold = self.get_parameter("move_hold_time").value
        self.joy_topic = self.get_parameter("joy_topic").value
        self.event_topic = self.get_parameter("control_event_topic").value
        self.emergency_topic = self.get_parameter("control_emergency_topic").value
        self.cmd_topic = str(self.get_parameter('cmd_topic').value)


        # ---------------- CONTROLLER ----------------
        self.ps4 = PS4Controller()

        # ---------------- STATE ----------------
        self.armed = False
        self.power = False
        self.light = False

        self._hold_start = {}
        self._prev = {}

        # ---------------- ROS ----------------
        self.pub_event = self.create_publisher(
            ControlEvents, self.event_topic, 30
        )
        self.pub_emergency = self.create_publisher(
            Emergency, self.emergency_topic, 10
        )

        self.pub_cmd_vel = self.create_publisher(
            Twist, self.cmd_topic, 30
        )

        self.create_subscription(Joy, self.joy_topic, self.joy_cb, 30)

        self.get_logger().info("🎮 PS4 node READY")

    # ==================================================
    # HELPERS
    # ==================================================

    def send_event(self, category, type_, value):
        msg = ControlEvents()
        msg.source = ControlEvents.SOURCE_PS4
        msg.category = category
        msg.type = type_
        msg.value = value
        msg.stamp = self.get_clock().now().to_msg()
        self.pub_event.publish(msg)

    def edge(self, name, now):
        prev = self._prev.get(name, False)
        self._prev[name] = now
        return now and not prev

    def hold(self, name, pressed, hold_time, now):
        if pressed:
            if name not in self._hold_start:
                self._hold_start[name] = now
            elif now - self._hold_start[name] >= hold_time:
                self._hold_start.pop(name)
                return True
        else:
            self._hold_start.pop(name, None)
        return False

    def send_if_changed(self, name, category, type_, value):
        prev = self._prev.get(name)
        if prev == value:
            return
        self._prev[name] = value
        self.send_event(category, type_, value)

    def publish_cmd_vel(self):
        msg = Twist()

        msg.linear.x = self.ps4.lin
        msg.linear.y = 0.0
        msg.linear.z = 0.0

        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = self.ps4.ang

        self.pub_cmd_vel.publish(msg)

    # ==================================================
    # MAIN CALLBACK
    # ==================================================

    def joy_cb(self, msg: Joy):
        now = time.monotonic()
        self.ps4.process_joy(msg.axes, msg.buttons)
        self.ps4.check_timeout()
        self.publish_cmd_vel()

        # ---------- EMERGENCY ----------
        if self.ps4.joystick_lost:
            em = Emergency()
            em.active = True
            em.level = Emergency.LEVEL_HARD
            em.reason = "ps4_timeout"
            em.stamp = self.get_clock().now().to_msg()
            self.pub_emergency.publish(em)
            return

        # ---------- ARM ----------
        if self.hold("X", self.ps4.X_btn, self.arm_hold, now):
            self.armed = not self.armed
            self.send_event(
                ControlEvents.CATEGORY_STATE,
                ControlEvents.STATE_ARMED,
                int(self.armed),
            )

        # ---------- POWER ----------
        if self.hold("O", self.ps4.O_btn, self.power_hold, now):
            self.power = not self.power
            self.send_event(
                ControlEvents.CATEGORY_STATE,
                ControlEvents.STATE_POWER,
                int(self.power),
            )

        # ---------- LIGHT MODE ----------
        if self.hold("SQ", self.ps4.Square_btn, self.light_hold, now):
            self.light = not self.light
            self.send_event(
                ControlEvents.CATEGORY_STATE,
                ControlEvents.STATE_LIGHT,
                int(self.light),
            )

        # ---------- EVENTS ----------
        self.send_if_changed(
            "ENGINE_START",
            ControlEvents.CATEGORY_EVENT,
            ControlEvents.ENGINE_START,
            int(self.ps4.Triangle_btn),
        )

        self.send_if_changed(
            "ENGINE_STOP",
            ControlEvents.CATEGORY_EVENT,
            ControlEvents.ENGINE_STOP,
            int(not self.armed),
        )

        self.send_if_changed(
            "CLUTCH",
            ControlEvents.CATEGORY_EVENT,
            ControlEvents.CLUTCH_ACTIVE,
            int(self.ps4.R1_btn),
        )

        self.send_if_changed(
            "BRAKE",
            ControlEvents.CATEGORY_EVENT,
            ControlEvents.BRAKE_ACTIVE,
            int(self.ps4.L1_btn),
        )

        if self.hold("MOVE", self.ps4.L1_btn, self.move_hold, now):
            self.send_if_changed(
                "MOVE_ALLOWED",
                ControlEvents.CATEGORY_EVENT,
                ControlEvents.MOVE_ALLOWED,
                int(self.ps4.L1_btn),
            )
        if not self.ps4.L1_btn:
            self.send_if_changed(
                "MOVE_ALLOWED",
                ControlEvents.CATEGORY_EVENT,
                ControlEvents.MOVE_ALLOWED,
                int(self.ps4.L1_btn),
            )

        # ---------- LIGHTS ----------
        if self.edge("ARMED_FP", self.armed):
            self.send_event(
                ControlEvents.CATEGORY_LIGHT,
                ControlEvents.FRONT_POSITION,
                int(self.armed),
            )

        if self.light and self.edge("UP", self.ps4.up_btn):
            self.send_event(
                ControlEvents.CATEGORY_LIGHT,
                ControlEvents.FRONT_SEQUENCE_NEXT,
                1,
            )

        if self.light and self.edge("DOWN", self.ps4.down_btn):
            self.send_event(
                ControlEvents.CATEGORY_LIGHT,
                ControlEvents.BACK_POSITION,
                1,
            )

        if self.light and self.edge("LEFT", self.ps4.left_btn):
            self.send_event(
                ControlEvents.CATEGORY_LIGHT,
                ControlEvents.LEFT_BLINK,
                1,
            )

        if self.light and self.edge("RIGHT", self.ps4.right_btn):
            self.send_event(
                ControlEvents.CATEGORY_LIGHT,
                ControlEvents.RIGHT_BLINK,
                1,
            )


def main():
    rclpy.init()
    node = PS4Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
