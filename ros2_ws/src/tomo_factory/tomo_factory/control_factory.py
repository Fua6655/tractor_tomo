#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from tomo_msgs.msg import ControlEvents, OutputStates, Emergency
from builtin_interfaces.msg import Time


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
        # INTERNAL STATE (REDUCER STATE)
        # ==================================================
        self.state = {
            # system
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

        # emergency handling
        self.emergency_active = False
        self.emergency_level = 0      # 0 = soft, 1 = hard
        self.saved_state = None       # used only for SOFT emergency

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
        """
        Handles SOFT vs HARD emergency.
        """

        # ---------------- HARD EMERGENCY ----------------
        if msg.active and msg.level == 1:
            self.get_logger().error("🚨 HARD EMERGENCY")

            self.emergency_active = True
            self.emergency_level = 1

            self._hard_zero()
            self.state["active_source"] = "EMERGENCY"

            self.publish()
            return

        # ---------------- SOFT EMERGENCY ----------------
        if msg.active and msg.level == 0:
            self.get_logger().warn("⚠️ SOFT EMERGENCY")

            if not self.emergency_active:
                self.saved_state = self.state.copy()

            self.emergency_active = True
            self.emergency_level = 0

            self._soft_zero()
            self.publish()
            return

        # ---------------- RELEASE ----------------
        if not msg.active and self.emergency_active:
            self.get_logger().info("✅ Emergency released")

            if self.emergency_level == 0 and self.saved_state:
                self.state = self.saved_state
                self.saved_state = None

            self.emergency_active = False
            self.emergency_level = 0

            self.publish()

    # ==================================================
    # EVENT REDUCER
    # ==================================================

    def event_cb(self, msg: ControlEvents):
        """
        Main reducer logic.
        """

        # ---------- HARD emergency blocks EVERYTHING ----------
        if self.emergency_active and self.emergency_level == 1:
            return

        # ---------- SOFT emergency blocks dangerous stuff ----------
        if self.emergency_active and self.emergency_level == 0:
            if msg.category in (
                ControlEvents.CAT_EVENT,
                ControlEvents.CAT_MOTION,
            ):
                return

        # ---------- SOURCE GUARD ----------
        src = self._source_to_string(msg.source)
        if src != self.state["active_source"]:
            # only SYSTEM events may change source
            if msg.category != ControlEvents.CAT_SYSTEM:
                return

        # ---------- REDUCE ----------
        self.reduce(msg)
        self.publish()

    # ==================================================
    # REDUCER IMPLEMENTATION
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
            mapping = {
                ControlEvents.FRONT_POSITION: "front_position",
                ControlEvents.FRONT_SHORT: "front_short",
                ControlEvents.FRONT_LONG: "front_long",
                ControlEvents.BACK_POSITION: "back_light",
                ControlEvents.LEFT_BLINK: "left_blink",
                ControlEvents.RIGHT_BLINK: "right_blink",
            }
            key = mapping.get(msg.type)
            if key:
                self.state[key] = bool(msg.value)

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
    # ZEROING HELPERS
    # ==================================================

    def _soft_zero(self):
        self.state["engine_start"] = False
        self.state["clutch_down"] = False
        self.state["move_allowed"] = False
        self.state["linear"] = 0.0
        self.state["angular"] = 0.0

    def _hard_zero(self):
        for k in self.state:
            if isinstance(self.state[k], bool):
                self.state[k] = False
            elif isinstance(self.state[k], float):
                self.state[k] = 0.0

    # ==================================================
    # PUBLISH OUTPUT
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
