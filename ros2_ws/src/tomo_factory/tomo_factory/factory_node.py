#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tomo_msgs.msg import ControlEvents, OutputStates, Emergency
from std_msgs.msg import String


class Factory(Node):

    def __init__(self):
        super().__init__("tomo_factory")

        # ---------------- STATE ----------------
        self.state = self._initial_state()

        # ---------------- SUBSCRIBERS ----------------
        self.create_subscription(
            ControlEvents,
            "control/event",
            self.event_cb,
            50
        )

        self.create_subscription(
            Emergency,
            "control/emergency",
            self.emergency_cb,
            10
        )

        # ---------------- PUBLISHERS ----------------
        self.pub_output = self.create_publisher(
            OutputStates,
            "tomo/output",
            50
        )

        self.pub_source = self.create_publisher(
            String,
            "factory/active_source",
            10
        )

        self.get_logger().info("✅ TOMO Factory READY")

    # ==================================================
    def _initial_state(self):
        return {
            "source": "PS4",
            "emergency": False,

            "armed": False,
            "power_mode": False,
            "light_mode": False,

            "engine_start": False,
            "clutch_down": False,
            "high_speed": False,
            "move_allowed": False,

            "front_position": False,
            "front_short": False,
            "front_long": False,
            "back_light": False,
            "left_blink": False,
            "right_blink": False,

            "linear": 0.0,
            "angular": 0.0,
        }

    # ==================================================
    def emergency_cb(self, msg: Emergency):
        self.state["emergency"] = msg.active

        if msg.active:
            self._hard_zero()
            self.state["source"] = "EMERGENCY"

        self.publish()

    # ==================================================
    def event_cb(self, msg: ControlEvents):
        if self.state["emergency"]:
            return

        # -------- SOURCE ARBITRATION --------
        src = {
            ControlEvents.SOURCE_PS4: "PS4",
            ControlEvents.SOURCE_WEB: "WEB",
            ControlEvents.SOURCE_AUTO: "AUTO",
        }.get(msg.source, "UNKNOWN")

        if src != self.state["source"]:
            return

        # -------- REDUCE --------
        self.reduce(msg)
        self.publish()

    # ==================================================
    def reduce(self, e: ControlEvents):

        c = e.category
        t = e.type
        v = bool(e.value)

        # ---------- STATES ----------
        if c == ControlEvents.CAT_STATE:
            if t == ControlEvents.STATE_ARMED:
                self.state["armed"] = v
            elif t == ControlEvents.STATE_POWER:
                self.state["power_mode"] = v
            elif t == ControlEvents.STATE_LIGHT:
                self.state["light_mode"] = v

        # ---------- EVENTS ----------
        elif c == ControlEvents.CAT_EVENT:
            if t == ControlEvents.ENGINE_START:
                self.state["engine_start"] = v
            elif t == ControlEvents.CLUTCH_ACTIVE:
                self.state["clutch_down"] = v
            elif t == ControlEvents.MOVE_ALLOWED:
                self.state["move_allowed"] = v

        # ---------- LIGHTS ----------
        elif c == ControlEvents.CAT_LIGHT:
            if t == ControlEvents.FRONT_POSITION:
                self.state["front_position"] = v
            elif t == ControlEvents.FRONT_SHORT:
                self.state["front_short"] = v
            elif t == ControlEvents.FRONT_LONG:
                self.state["front_long"] = v
            elif t == ControlEvents.BACK_POSITION:
                self.state["back_light"] = v
            elif t == ControlEvents.LEFT_BLINK:
                self.state["left_blink"] = v
            elif t == ControlEvents.RIGHT_BLINK:
                self.state["right_blink"] = v

        # ---------- MOTION ----------
        elif c == ControlEvents.CAT_MOTION:
            self.state["linear"] = e.linear
            self.state["angular"] = e.angular

        # ---------- SYSTEM ----------
        elif c == ControlEvents.CAT_SYSTEM:
            if t == ControlEvents.SYS_FORCE_SOURCE:
                self.state["source"] = ["PS4", "WEB", "AUTO"][e.value]

    # ==================================================
    def _hard_zero(self):
        for k in self.state:
            if isinstance(self.state[k], bool):
                self.state[k] = False
            if isinstance(self.state[k], float):
                self.state[k] = 0.0

    # ==================================================
    def publish(self):
        msg = OutputStates()

        msg.emergency = self.state["emergency"]
        msg.armed = self.state["armed"]
        msg.power_mode = self.state["power_mode"]
        msg.light_mode = self.state["light_mode"]

        msg.engine_start = self.state["engine_start"]
        msg.clutch_down = self.state["clutch_down"]
        msg.high_speed = self.state["high_speed"]
        msg.move_allowed = self.state["move_allowed"]

        msg.front_position = self.state["front_position"]
        msg.front_short = self.state["front_short"]
        msg.front_long = self.state["front_long"]
        msg.back_light = self.state["back_light"]
        msg.left_blink = self.state["left_blink"]
        msg.right_blink = self.state["right_blink"]

        msg.linear = self.state["linear"]
        msg.angular = self.state["angular"]

        msg.active_source = self.state["source"]
        msg.stamp = self.get_clock().now().to_msg()

        self.pub_output.publish(msg)
        self.pub_source.publish(String(data=self.state["source"]))


def main():
    rclpy.init()
    rclpy.spin(Factory())
    rclpy.shutdown()

