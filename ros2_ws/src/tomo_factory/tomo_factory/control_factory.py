#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from tomo_msgs.msg import ControlEvents, OutputStates, Emergency
from geometry_msgs.msg import Twist


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

        # ---------------- PARAMETERS ----------------
        self.declare_parameter('control_event_topic', '/control/events')
        self.declare_parameter('control_emergency_topic', '/control/emergency')
        self.declare_parameter('ps4_cmd_topic', '/ps4/cmd_vel')
        self.declare_parameter('auto_cmd_topic', '/auto/cmd_vel')
        self.declare_parameter('output_topic', '/tomo/states')
        self.declare_parameter('output_cmd_topic', 'tomo/cmd_vel')
        self.declare_parameter('ps4_timeout', 0.5)
        self.declare_parameter('web_timeout', 1.0)
        self.declare_parameter('auto_timeout', 0.5)


        self.control_event_topic = self.get_parameter("control_event_topic").value
        self.control_emergency_topic = self.get_parameter("control_emergency_topic").value
        self.ps4_cmd_topic = self.get_parameter("ps4_cmd_topic").value
        self.auto_cmd_topic = self.get_parameter("auto_cmd_topic").value
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.output_cmd_topic = str(self.get_parameter('output_cmd_topic').value)
        self.ps4_timeout = float(self.get_parameter('ps4_timeout').value)
        self.web_timeout = float(self.get_parameter('web_timeout').value)
        self.auto_timeout = float(self.get_parameter('auto_timeout').value)

        # ==================================================
        # REDUCER STATE (SINGLE SOURCE OF TRUTH)
        # ==================================================
        self.state = {
            "active_source": "PS4",
            "emergency": False,

            # states
            "armed_state": False,
            "power_state": False,
            "light_state": False,

            # events
            "engine_start": False,
            "engine_stop": False,
            "clutch_active": False,
            "brake_active": False,
            "move_allowed": False,

            # lights
            "front_position": False,
            "front_short": False,
            "front_long": False,
            "back_position": False,
            "left_blink": False,
            "right_blink": False,
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
        self.create_subscription(ControlEvents,self.control_event_topic,self.event_cb,50)
        self.create_subscription(Emergency,self.control_emergency_topic,self.emergency_cb,10)

        self.pub_output = self.create_publisher(OutputStates,self.output_topic,10)
        self.pub_output_cmd_vel = self.create_publisher(Twist, self.output_cmd_topic, 10)

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
                ControlEvents.CATEGORY_EVENT,
                #ControlEvents.CATEGORY_MOTION,
            ):
                return

        # SOURCE GUARD
        src = self._source_to_string(msg.source)
        if src != self.state["active_source"]:
            if msg.category != ControlEvents.CATEGORY_SYSTEM:
                return

        self.reduce(msg)
        self.publish()

    # ==================================================
    # REDUCER
    # ==================================================

    def reduce(self, msg: ControlEvents):

        # ---------- STATES ----------
        if msg.category == ControlEvents.CATEGORY_STATE:
            if msg.type == ControlEvents.STATE_ARMED:
                self.state["armed_state"] = bool(msg.value)
            elif msg.type == ControlEvents.STATE_POWER:
                self.state["power_state"] = bool(msg.value)
            elif msg.type == ControlEvents.STATE_LIGHT:
                self.state["light_state"] = bool(msg.value)

        # ---------- EVENTS ----------
        elif msg.category == ControlEvents.CATEGORY_EVENT:
            if msg.type == ControlEvents.ENGINE_START:
                self.state["engine_start"] = bool(msg.value)
            elif msg.type == ControlEvents.ENGINE_STOP:
                self.state["engine_stop"] = bool(msg.value)
            elif msg.type == ControlEvents.CLUTCH_ACTIVE:
                self.state["clutch_active"] = bool(msg.value)
            elif msg.type == ControlEvents.BRAKE_ACTIVE:
                self.state["brake_active"] = bool(msg.value)
            elif msg.type == ControlEvents.MOVE_ALLOWED:
                self.state["move_allowed"] = bool(msg.value)

        # ---------- LIGHTS ----------
        elif msg.category == ControlEvents.CATEGORY_LIGHT:

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
                self.state["back_position"] = not self.state["back_position"]

        # ---------- MOTION ----------
        #elif msg.category == ControlEvents.CATEGORY_MOTION:
         #   self.state["linear"] = msg.linear
          #  self.state["angular"] = msg.angular

        # ---------- SYSTEM ----------
        elif msg.category == ControlEvents.CATEGORY_SYSTEM:
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

        new_left = self.blink_left_active and self.blink_phase
        new_right = self.blink_right_active and self.blink_phase

        if (
                new_left != self.state["left_blink"] or
                new_right != self.state["right_blink"]
        ):
            self.state["left_blink"] = new_left
            self.state["right_blink"] = new_right
            self.publish()

    # ==================================================
    # ZEROING
    # ==================================================

    def _soft_zero(self):
        self.state["engine_start"] = False
        self.state["engine_stop"] = False
        self.state["clutch_active"] = False
        self.state["brake_active"] = False
        self.state["move_allowed"] = False
        #self.state["linear"] = 0.0
        #self.state["angular"] = 0.0

    def _hard_zero(self):
        #todo soft zero +
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

        msg.armed_state = self.state["armed_state"]
        msg.power_state = self.state["power_state"]
        msg.light_state= self.state["light_state"]

        msg.engine_start = self.state["engine_start"]
        msg.engine_stop = self.state["engine_stop"]
        msg.clutch_active = self.state["clutch_active"]
        msg.brake_active = self.state["brake_active"]
        msg.move_allowed = self.state["move_allowed"]

        msg.front_position = self.state["front_position"]
        msg.front_short = self.state["front_short"]
        msg.front_long = self.state["front_long"]
        msg.back_position = self.state["back_position"]
        msg.left_blink = self.state["left_blink"]
        msg.right_blink = self.state["right_blink"]

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
