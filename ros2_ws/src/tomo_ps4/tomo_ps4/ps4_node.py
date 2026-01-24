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
    """
    PS4 → ControlEvents
    Event-based, no direct control of outputs.
    """

    def __init__(self):
        super().__init__("ps4_node")

        # ---------------- PARAMETERS ----------------
        self.declare_parameter("arm_hold_time", 2.0)
        self.declare_parameter("deadzone", 0.08)
        self.declare_parameter("linear_scale", 0.6)
        self.declare_parameter("angular_scale", 1.2)
        self.declare_parameter('cmd_topic', '/ps4/cmd_vel')
        self.declare_parameter('joy_topic', '/joy')

        self.arm_hold_time = self.get_parameter("arm_hold_time").value
        self.deadzone = self.get_parameter("deadzone").value
        self.linear_scale = self.get_parameter("linear_scale").value
        self.angular_scale = self.get_parameter("angular_scale").value
        self.cmd_topic = str(self.get_parameter('cmd_topic').value)
        self.joy_topic = str(self.get_parameter('joy_topic').value)

        # ---------------- CONTROLLER ----------------
        self.ps4 = PS4Controller()

        # ---------------- STATE ----------------
        self.armed = False
        self._x_press_time = None
        self._prev = {}

        # ---------------- PUB / SUB ----------------
        self.pub_event = self.create_publisher(ControlEvents,"control/event",50)
        self.pub_emergency = self.create_publisher(Emergency,"control/emergency",10)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)

        self.create_subscription(Joy,self.joy_topic,self.joy_cb,50)

        self.get_logger().info("🎮 PS4 node READY")

    # ==================================================
    # HELPERS
    # ==================================================

    def send_event(
        self,
        category: int,
        type_: int,
        value: int = 0,
    ):
        msg = ControlEvents()
        msg.source = ControlEvents.SOURCE_PS4
        msg.category = category
        msg.type = type_
        msg.value = value
        msg.stamp = self.get_clock().now().to_msg()
        self.pub_event.publish(msg)

    def edge(self, name: str, now: bool) -> bool:
        prev = self._prev.get(name, False)
        self._prev[name] = now
        return now and not prev

    def apply_deadzone(self, v: float) -> float:
        return 0.0 if abs(v) < self.deadzone else v

    # ==================================================
    # MAIN CALLBACK
    # ==================================================

    def joy_cb(self, msg: Joy):
        axes: List[float] = list(msg.axes)
        buttons: List[int] = list(msg.buttons)
        now = time.monotonic()

        self.ps4.process_joy(axes, buttons)
        self.ps4.check_timeout()

        # --------------------------------------------------
        # EMERGENCY (PS BUTTON or timeout)
        # --------------------------------------------------
        if self.ps4.joystick_lost:
            em = Emergency()
            em.active = True
            em.level = Emergency.LEVEL_HARD
            em.reason = "ps4_timeout"
            em.stamp = self.get_clock().now().to_msg()
            self.pub_emergency.publish(em)
            return

        # --------------------------------------------------
        # ARM / DISARM (X HOLD)
        # --------------------------------------------------
        x = bool(self.ps4.X_btn)

        if x and self._x_press_time is None:
            self._x_press_time = now

        if (
            x
            and self._x_press_time
            and now - self._x_press_time >= self.arm_hold_time
        ):
            self.armed = not self.armed
            self.send_event(
                ControlEvents.CATEGORY_STATE,
                ControlEvents.STATE_ARMED,
                int(self.armed),
            )
            self.get_logger().info(
                f"{'ARMED' if self.armed else 'DISARMED'}"
            )
            self._x_press_time = None

        if not x:
            self._x_press_time = None

        # --------------------------------------------------
        # POWER MODE (O toggle)
        # --------------------------------------------------
        if self.edge("O", bool(self.ps4.O_btn)) and self.armed:
            self.send_event(
                ControlEvents.CATEGORY_STATE,
                ControlEvents.STATE_POWER,
                1,
            )

        # --------------------------------------------------
        # LIGHT MODE (SQUARE toggle)
        # --------------------------------------------------
        if self.edge("SQUARE", bool(self.ps4.Square_btn)) and self.armed:
            self.send_event(
                ControlEvents.CATEGORY_STATE,
                ControlEvents.STATE_LIGHT,
                1,
            )

        # --------------------------------------------------
        # EVENTS
        # --------------------------------------------------
        self.send_event(
            ControlEvents.CATEGORY_EVENT,
            ControlEvents.ENGINE_START,
            int(self.ps4.Triangle_btn and self.armed),
        )

        self.send_event(
            ControlEvents.CATEGORY_EVENT,
            ControlEvents.ENGINE_STOP,
            int(self.ps4.R1_btn and self.armed),
        )

        self.send_event(
            ControlEvents.CATEGORY_EVENT,
            ControlEvents.CLUTCH_ACTIVE,
            int(self.ps4.R1_btn and self.armed),
        )

        self.send_event(
            ControlEvents.CATEGORY_EVENT,
            ControlEvents.BRAKE_ACTIVE,
            int(self.ps4.R1_btn and self.armed),
        )

        self.send_event(
            ControlEvents.CATEGORY_EVENT,
            ControlEvents.MOVE_ALLOWED,
            int(self.ps4.L1_btn and self.armed),
        )

        # --------------------------------------------------
        # LIGHTS (DPAD)
        # --------------------------------------------------
        if self.edge("UP", bool(self.ps4.up_btn)):
            self.send_event(
                ControlEvents.CATEGORY_LIGHT,
                ControlEvents.FRONT_SHORT,
                1,
            )

        if self.edge("DOWN", bool(self.ps4.down_btn)):
            self.send_event(
                ControlEvents.CATEGORY_LIGHT,
                ControlEvents.BACK_POSITION,
                1,
            )

        if self.edge("LEFT", bool(self.ps4.left_btn)):
            self.send_event(
                ControlEvents.CATEGORY_LIGHT,
                ControlEvents.LEFT_BLINK,
                1,
            )

        if self.edge("RIGHT", bool(self.ps4.right_btn)):
            self.send_event(
                ControlEvents.CATEGORY_LIGHT,
                ControlEvents.RIGHT_BLINK,
                1,
            )

        # --------------------------------------------------
        # MOTION
        # --------------------------------------------------
        lin = self.apply_deadzone(self.ps4.joy_left_y) * self.linear_scale
        ang = self.apply_deadzone(self.ps4.joy_right_x) * self.angular_scale

        twist = Twist()
        twist.linear.x = lin
        twist.angular.z = ang
        self.cmd_pub.publish(twist)


def main():
    rclpy.init()
    node = PS4Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
