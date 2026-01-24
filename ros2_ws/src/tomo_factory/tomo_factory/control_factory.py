#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from tomo_msgs.msg import ControlEvents, OutputStates, Emergency


class ControlFactory(Node):
    """
    Central reducer + state machine.

    INPUT:
      - ControlEvents  (ps4 / web / auto)
      - Emergency      (soft / hard)

    OUTPUT:
      - OutputStates   (single authoritative state for ESP)
    """

    def __init__(self):
        super().__init__("control_factory")

        # ==================================================
        # REDUCER STATE (SINGLE SOURCE OF TRUTH)
        # ==================================================
        self.state = {
            "active_source": "PS4",
            "emergency": False,

            # states
            "armed": False,
            "power_mode": False,
            "light_mode": False,

            # events
            "engine_start": False,
            "clutch_down": False,
            "high_speed": False,
            "move_allowed": False,

            # lights
            "front_position": False,
            "front_short": False,
            "front_long": False,
            "back_light": False,
            "left_blink": False,
            "right_blink": False,

            # motion
            "linear": 0.0,
            "angular": 0.0,
        }

        # ==================================================
        # EMERGENCY
        # ==================================================
        self.emergency_active = False
        self.emergency_level = 0   # 0 = soft, 1 = hard
        self.saved_state = None

        # ==================================================
        # BLINKER INTERNAL STATE
        # ==================================================
        self.blink_left_active = False
        self.blink_right_active = False
        self.blink_phase = False

        self.create_timer(0.5, self._blink_timer)

        # ==================================================
        # ROS INTERFACES
        # ==================================================
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

        self.pub_output = self.create_publisher(
            OutputStates,
            "tomo/output",
            10
        )

        self.get_logger().info("✅ ControlFactory READY (event-based)")

    # ==================================================
    # EMERGENCY HANDLING
    # ==================================================

    def emergency_cb(self, msg: Emergency):

        # -------- HARD EMERGENCY --------
        if msg.active and msg.level == 1:
            self.get_logger().error("🚨 HARD EMERGENCY")

            self.emergency_active = True
            self.emergency_level = 1

            self._hard_zero()
            self.state["active_source"] = "EMERGENCY"

            self.publish()
            return

        # -------- SOFT EMERGENCY --------
        if msg.active and msg.level == 0:
            self.get_logger().warn("⚠️ SOFT EMERGENCY")

            if not self.emergency_active:
                self.saved_state = self.state.copy()

            self.emergency_active = True
            self.emergency_level = 0

            self._soft_zero()
            self.publish()
            return

        # -------- RELEASE --------
        if not msg.active and self.emergency_active:
            self.get_logger().info("✅ Emergency released")

            if self.emergency_level == 0 and self.saved_state:
                self.state = self.saved_state
                self.saved_state = None

            self.emergency_active = False
            self.emergency_level = 0

            self.publish()

    # ==================================================
    # EVENT CALLBACK (REDUCER ENTRY)
    # ==================================================

    def event_cb(self, msg: ControlEvents):

        # HARD emergency blocks everything
        if self.emergency_active and self.emergency_level == 1:
            return

        # SOFT emergency blocks dangerous stuff
        if self.emergency_active and self.emergency_level == 0:
            if msg.category in (
                ControlEvents.CAT_EVENT,
                ControlEvents.CAT_MOTION,
            ):
                return

        # SOURCE GUARD
        src = self._source_to_string(msg.source)
        if src != self.state["active_source"]:
            if msg.category != ControlEvents.CAT_SYSTEM:
                return

        self.reduce(msg)
        self.publish()

    # ==================================================
    # REDUCER
    # ==================================================

    def reduce(self, msg: ControlEvents):

        # ---------- STATES ----------
        if msg.category == ControlEvents.CAT_STATE:
            if msg.type == ControlEvents.STATE_ARMED:
                self.state["armed"] = bool(msg.value)
            elif msg.type == ControlEvents.STATE_POWER:
                self.state["power_mode"] = bool(msg.value)
            elif msg.type == ControlEvents.STATE_LIGHT:
                self.state["light_mode"] = bool(msg.value)

        # ---------- EVENTS ----------
        elif msg.category == ControlEvents.CAT_EVENT:
            if msg.type == ControlEvents.ENGINE_START:
                self.state["engine_start"] = bool(msg.value)
            elif msg.type == ControlEvents.CLUTCH_ACTIVE:
                self.state["clutch_down"] = bool(msg.value)
            elif msg.type == ControlEvents.MOVE_ALLOWED:
                self.state["move_allowed"] = bool(msg.value)

        # ---------- LIGHTS ----------
        elif msg.category == ControlEvents.CAT_LIGHT:

            if msg.type == ControlEvents.LEFT_BLINK:
                self.blink_left_active = not self.blink_left_active
                if not self.blink_left_active:
                    self.state["left_blink"] = False

            elif msg.type == ControlEvents.RIGHT_BLINK:
                self.blink_right_active = not self.blink_right_active
                if not self.blink_right_active:
                    self.state["right_blink"] = False

            elif msg.type == ControlEvents.FRONT_POSITION:
                self.state["front_position"] = not self.state["front_position"]

            elif msg.type == ControlEvents.FRONT_SHORT:
                self.state["front_short"] = not self.state["front_short"]

            elif msg.type == ControlEvents.FRONT_LONG:
                self.state["front_long"] = not self.state["front_long"]

            elif msg.type == ControlEvents.BACK_POSITION:
                self.state["back_light"] = not self.state["back_light"]

        # ---------- MOTION ----------
        elif msg.category == ControlEvents.CAT_MOTION:
            self.state["linear"] = msg.linear
            self.state["angular"] = msg.angular

        # ---------- SYSTEM ----------
        elif msg.category == ControlEvents.CAT_SYSTEM:
            if msg.type == ControlEvents.SYS_FORCE_SOURCE:
                self.state["active_source"] = self._source_to_string(msg.value)
            elif msg.type == ControlEvents.SYS_RESET:
                self._hard_zero()

    # ==================================================
    # BLINK TIMER
    # ==================================================

    def _blink_timer(self):
        if not (self.blink_left_active or self.blink_right_active):
            return

        self.blink_phase = not self.blink_phase

        self.state["left_blink"] = (
            self.blink_left_active and self.blink_phase
        )
        self.state["right_blink"] = (
            self.blink_right_active and self.blink_phase
        )

        self.publish()

    # ==================================================
    # ZEROING
    # ==================================================

    def _soft_zero(self):
        self.state["engine_start"] = False
        self.state["clutch_down"] = False
        self.state["move_allowed"] = False
        self.state["linear"] = 0.0
        self.state["angular"] = 0.0

    def _hard_zero(self):
        for k, v in self.state.items():
            if isinstance(v, bool):
                self.state[k] = False
            elif isinstance(v, float):
                self.state[k] = 0.0

        self.blink_left_active = False
        self.blink_right_active = False
        self.blink_phase = False

    # ==================================================
    # PUBLISH
    # ==================================================

    def publish(self):
        msg = OutputStates()

        msg.emergency = self.emergency_active
        msg.active_source = self.state["active_source"]

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

        msg.stamp = self.get_clock().now().to_msg()

        self.pub_output.publish(msg)

    # ==================================================
    # UTILS
    # ==================================================

    @staticmethod
    def _source_to_string(src: int) -> str:
        if src == ControlEvents.SOURCE_PS4:
            return "PS4"
        if src == ControlEvents.SOURCE_WEB:
            return "WEB"
        if src == ControlEvents.SOURCE_AUTO:
            return "AUTO"
        return "UNKNOWN"


def main():
    rclpy.init()
    node = ControlFactory()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
