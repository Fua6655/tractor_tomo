#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from enum import Enum

from tomo_msgs.msg import ControlEvents, OutputStates, Emergency


# ==================================================
# ENUMS
# ==================================================

class ArmState(Enum):
    DISARMED = 0
    ARMED = 1


class PowerState(Enum):
    OFF = 0
    ON = 1


class LightState(Enum):
    OFF = 0
    ON = 1

class ActiveSource(Enum):
    PS4 = ControlEvents.SOURCE_PS4
    WEB = ControlEvents.SOURCE_WEB
    AUTO = ControlEvents.SOURCE_AUTO

# ==================================================
# CONTROL FACTORY
# ==================================================

class ControlFactory(Node):

    def __init__(self):
        super().__init__("control_factory")

        self.declare_parameter("control_event_topic", "/control/events")
        self.declare_parameter("control_emergency_topic", "/control/emergency")
        self.declare_parameter("output_topic", "/tomo/states")

        self.event_topic = self.get_parameter("control_event_topic").value
        self.emergency_topic = self.get_parameter("control_emergency_topic").value
        self.output_topic = self.get_parameter("output_topic").value

        self.active_source = ActiveSource.PS4

        self.arm_state = ArmState.DISARMED
        self.power_state = PowerState.OFF
        self.light_state = LightState.OFF

        self.state = {
            "engine_start": False,
            "engine_stop": True,

            "clutch_active": False,
            "brake_active": False,
            "move_allowed": False,

            "front_position": False,
            "front_short": False,
            "front_long": False,
            "back_position": False,
            "left_blink": False,
            "right_blink": False,
        }

        self.front_mode = 0  # 0=OFF, 1=SHORT, 2=LONG

        # BLINKERS
        self.blink_left_active = False
        self.blink_right_active = False
        self.blink_phase = False
        self.blink_period = 0.5
        self.blink_timer = self.create_timer(self.blink_period, self._blink_timer)


        self.emergency_active = False
        self.emergency_level = 0  # 0 none, 1 soft, 2 hard

        self.saved_state = None
        self.saved_arm = None
        self.saved_power = None
        self.saved_light = None
        self.saved_front_mode = None
        self.saved_blink_l = None
        self.saved_blink_r = None

        self.create_subscription(ControlEvents, self.event_topic, self.event_cb, 50)
        self.create_subscription(Emergency, self.emergency_topic, self.emergency_cb, 10)
        self.pub_output = self.create_publisher(OutputStates, self.output_topic, 10)

        self.get_logger().info("✅ ControlFactory FINAL (ENGINE_START level-based)")

    # ==================================================
    # EMERGENCY HANDLING
    # ==================================================

    def emergency_cb(self, msg: Emergency):

        # -------- HARD EMERGENCY --------
        if msg.active and msg.level == Emergency.LEVEL_HARD:
            self.get_logger().error("🚨 HARD EMERGENCY")

            self.emergency_active = True
            self.emergency_level = Emergency.LEVEL_HARD
            self._set_blink_period(0.25)

            armed = self.arm_state
            front_pos = self.state["front_position"]

            self._hard_reset()

            self.arm_state = armed
            self.state["front_position"] = front_pos
            self.state["engine_stop"] = True

            self.publish()
            return

        # -------- SOFT EMERGENCY --------
        if msg.active and msg.level == Emergency.LEVEL_SOFT:
            self.get_logger().warn("⚠️ SOFT EMERGENCY")

            if not self.emergency_active:
                self._save_soft_state()

            self.emergency_active = True
            self.emergency_level = Emergency.LEVEL_SOFT
            self._set_blink_period(0.5)

            # block dangerous stuff
            self.state["engine_start"] = False
            self.state["move_allowed"] = False
            self.state["clutch_active"] = False
            self.state["brake_active"] = False
            self.power_state = PowerState.OFF
            self.state["engine_stop"] = True

            self.publish()
            return

        # -------- RELEASE --------
        if not msg.active and self.emergency_active:
            self.get_logger().info("✅ Emergency released")

            # HARD emergency release → clear engine stop
            if self.emergency_level == Emergency.LEVEL_HARD:
                self.state["engine_stop"] = False

            if self.emergency_level == Emergency.LEVEL_SOFT:
                self._restore_soft_state()

            self.emergency_active = False
            self.emergency_level = 0
            self._set_blink_period(0.5)

            self.publish()

    # ==================================================
    # EVENT CALLBACK
    # ==================================================

    def event_cb(self, msg: ControlEvents):

        # ---------- SYSTEM: FORCE SOURCE ----------
        if (
                msg.category == ControlEvents.CATEGORY_SYSTEM and
                msg.type == ControlEvents.SYS_FORCE_SOURCE
        ):
            try:
                requested = ActiveSource(msg.value)
                if self.active_source != requested:
                    self.active_source = requested
                    self.get_logger().info(f"🔁 Active source set to {requested.name}")
            except ValueError:
                self.get_logger().warn("⚠️ Invalid source value")

            self.publish()
            return

        # ---------- FILTER BY ACTIVE SOURCE ----------
        if msg.source != self.active_source.value:
            return

        # ---------- EMERGENCY GUARDS ----------
        if self.emergency_active and self.emergency_level == Emergency.LEVEL_HARD:
            return

        if self.emergency_active and self.emergency_level == Emergency.LEVEL_SOFT:
            if msg.category == ControlEvents.CATEGORY_EVENT:
                return

        self.reduce(msg)
        self.publish()

    # ==================================================
    # REDUCER
    # ==================================================

    def reduce(self, msg: ControlEvents):

        # DISARMED
        if self.arm_state == ArmState.DISARMED:
            self.state["engine_stop"] = True
            if (
                msg.category == ControlEvents.CATEGORY_STATE and
                msg.type == ControlEvents.STATE_ARMED and
                msg.value
            ):
                self._enter_armed()
            return

        # STATES
        if msg.category == ControlEvents.CATEGORY_STATE:

            # ==================================================
            # PS4 → FORCE / LEVEL BASED
            # ==================================================
            if msg.source == ControlEvents.SOURCE_PS4:

                if msg.type == ControlEvents.STATE_ARMED:
                    if msg.value:
                        self._enter_armed()
                    else:
                        self._exit_armed()
                    return

                if msg.type == ControlEvents.STATE_POWER:
                    if msg.value:
                        self._enter_power()
                    else:
                        self._exit_power()
                    return

                if msg.type == ControlEvents.STATE_LIGHT:
                    self.light_state = (
                        LightState.ON if msg.value else LightState.OFF
                    )
                    return

            # ==================================================
            # WEB / AUTO → TOGGLE BASED
            # ==================================================
            else:

                if msg.type == ControlEvents.STATE_ARMED:
                    if self.arm_state == ArmState.DISARMED:
                        self._enter_armed()
                    else:
                        self._exit_armed()
                    return

                if msg.type == ControlEvents.STATE_POWER:
                    if self.power_state == PowerState.OFF:
                        self._enter_power()
                    else:
                        self._exit_power()
                    return

                if msg.type == ControlEvents.STATE_LIGHT:
                    self.light_state = (
                        LightState.OFF
                        if self.light_state == LightState.ON
                        else LightState.ON
                    )
                    return

        # BLOCK LIGHT COMMANDS
        if msg.category == ControlEvents.CATEGORY_LIGHT and self.light_state != LightState.ON:
            return

        # EVENTS
        if msg.category == ControlEvents.CATEGORY_EVENT:

            # ==================================================
            # PS4 → LEVEL-BASED (HOLD)
            # ==================================================
            if msg.source == ControlEvents.SOURCE_PS4:

                if msg.type == ControlEvents.ENGINE_START:
                    self.state["engine_start"] = (
                            bool(msg.value) and self.power_state == PowerState.ON
                    )

                elif msg.type == ControlEvents.MOVE_ALLOWED:
                    self.state["move_allowed"] = bool(msg.value)

                elif msg.type == ControlEvents.CLUTCH_ACTIVE:
                    self.state["clutch_active"] = bool(msg.value)

                elif msg.type == ControlEvents.BRAKE_ACTIVE:
                    self.state["brake_active"] = bool(msg.value)

            # ==================================================
            # WEB / AUTO → TOGGLE (CLICK)
            # ==================================================
            else:

                if msg.type == ControlEvents.ENGINE_START:
                    if self.power_state == PowerState.ON:
                        self.state["engine_start"] = not self.state["engine_start"]

                elif msg.type == ControlEvents.ENGINE_STOP:
                        self.state["engine_stop"] = not self.state["engine_stop"]

                elif msg.type == ControlEvents.MOVE_ALLOWED:
                    self.state["move_allowed"] = not self.state["move_allowed"]

                elif msg.type == ControlEvents.CLUTCH_ACTIVE:
                    self.state["clutch_active"] = not self.state["clutch_active"]

                elif msg.type == ControlEvents.BRAKE_ACTIVE:
                    self.state["brake_active"] = not self.state["brake_active"]

        # LIGHTS
        elif msg.category == ControlEvents.CATEGORY_LIGHT:

            # ==================================================
            # PS4 → SEQUENCE BASED
            # ==================================================
            if msg.source == ControlEvents.SOURCE_PS4:

                if msg.type == ControlEvents.FRONT_SEQUENCE_NEXT:
                    self.front_mode = (self.front_mode + 1) % 3

                    self.state["front_short"] = self.front_mode == 1
                    self.state["front_long"] = self.front_mode == 2

                elif msg.type == ControlEvents.BACK_POSITION:
                    self.state["back_position"] = not self.state["back_position"]

                elif msg.type == ControlEvents.LEFT_BLINK:
                    self.blink_left_active = not self.blink_left_active

                elif msg.type == ControlEvents.RIGHT_BLINK:
                    self.blink_right_active = not self.blink_right_active

            # ==================================================
            # WEB / AUTO → DIRECT TOGGLE
            # ==================================================
            else:

                if msg.type == ControlEvents.FRONT_SHORT:
                    self.state["front_short"] = not self.state["front_short"]
                    if self.state["front_short"]:
                        self.state["front_long"] = False

                elif msg.type == ControlEvents.FRONT_LONG:
                    self.state["front_long"] = not self.state["front_long"]
                    if self.state["front_long"]:
                        self.state["front_short"] = False

                elif msg.type == ControlEvents.BACK_POSITION:
                    self.state["back_position"] = not self.state["back_position"]

                elif msg.type == ControlEvents.LEFT_BLINK:
                    self.blink_left_active = not self.blink_left_active

                elif msg.type == ControlEvents.RIGHT_BLINK:
                    self.blink_right_active = not self.blink_right_active

    # ==================================================
    # FSM TRANSITIONS
    # ==================================================

    def _enter_armed(self):
        self.arm_state = ArmState.ARMED
        self.state["engine_stop"] = False
        self.state["front_position"] = True

    def _exit_armed(self):
        self._hard_reset()
        self.arm_state = ArmState.DISARMED
        self.state["engine_stop"] = True

    def _enter_power(self):
        if self.power_state == PowerState.ON:
            return
        self.power_state = PowerState.ON
        self.state["engine_start"] = False

    def _exit_power(self):
        self.power_state = PowerState.OFF
        self.state["engine_start"] = False

    # ==================================================
    # BLINK TIMER
    # ==================================================

    def _blink_timer(self):
        self.blink_phase = not self.blink_phase

        if self.emergency_active:
            self.state["left_blink"] = self.blink_phase
            self.state["right_blink"] = self.blink_phase
        else:
            self.state["left_blink"] = self.blink_left_active and self.blink_phase
            self.state["right_blink"] = self.blink_right_active and self.blink_phase

        self.publish()

    def _set_blink_period(self, period):
        if self.blink_period == period:
            return
        self.blink_period = period
        self.blink_timer.cancel()
        self.blink_timer = self.create_timer(period, self._blink_timer)

    # ==================================================
    # SOFT SAVE / RESTORE
    # ==================================================

    def _save_soft_state(self):
        self.saved_state = self.state.copy()
        self.saved_arm = self.arm_state
        self.saved_power = self.power_state
        self.saved_light = self.light_state
        self.saved_front_mode = self.front_mode
        self.saved_blink_l = self.blink_left_active
        self.saved_blink_r = self.blink_right_active

    def _restore_soft_state(self):
        if not self.saved_state:
            return

        self.state = self.saved_state
        self.arm_state = self.saved_arm
        self.power_state = self.saved_power
        self.light_state = self.saved_light
        self.front_mode = self.saved_front_mode
        self.blink_left_active = self.saved_blink_l
        self.blink_right_active = self.saved_blink_r

        self.saved_state = None

    # ==================================================
    # HARD RESET
    # ==================================================

    def _hard_reset(self):
        for k in self.state:
            self.state[k] = False

        self.state["engine_stop"] = True
        self.front_mode = 0
        self.power_state = PowerState.OFF
        self.light_state = LightState.OFF
        self.blink_left_active = False
        self.blink_right_active = False

    # ==================================================
    # PUBLISH
    # ==================================================

    def publish(self):
        msg = OutputStates()

        msg.source = self.active_source.value

        msg.armed_state = self.arm_state == ArmState.ARMED
        msg.power_state = self.power_state == PowerState.ON
        msg.light_state = self.light_state == LightState.ON

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


def main():
    rclpy.init()
    node = ControlFactory()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
