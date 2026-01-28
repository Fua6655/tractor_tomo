# ps4_controller.py
from typing import List, Optional, Union
import time


class PS4Controller:
    """
    Unified PS4-only joystick mapping (Python port from your original structure) with failsafe.
    """
    def __init__(self):
        self.up_btn = 0
        self.down_btn = 0
        self.left_btn = 0
        self.right_btn = 0

        self.X_btn = 0
        self.O_btn = 0
        self.Triangle_btn = 0
        self.Square_btn = 0

        self.R1_btn = 0
        self.L1_btn = 0
        self.R2_btn = 0
        self.L2_btn = 0

        self.R2 = 1.0
        self.L2 = 1.0

        self.d_pad_x = 0
        self.d_pad_y = 0

        self.joy_left_x = 0.0
        self.joy_left_y = 0.0
        self.joy_right_x = 0.0
        self.joy_right_y = 0.0

        self._last_axes: Optional[List[float]] = None
        self._last_buttons: Optional[List[Union[int, float]]] = None

        self.last_joy_time = time.monotonic()
        self.timeout = 1.0
        self.joystick_lost = False

        self.deadzone = 0.1
        self.lin = 0.0
        self.ang = 0.0

    def reset(self):
        self.__init__()

    def apply_deadzone(self, v: float) -> float:
        if abs(v) < self.deadzone:
            return 0.0
        return v

    def process_joy(self, axes: List[float], buttons: List[Union[int, float]]):
        self._last_axes = list(axes)
        self._last_buttons = list(buttons)
        self.last_joy_time = time.monotonic()
        self.joystick_lost = False

        def a(i, default=0.0):
            return axes[i] if 0 <= i < len(axes) else default
        def b(i, default=0):
            return int(buttons[i]) if 0 <= i < len(buttons) else default

        self.joy_left_x = float(a(0))
        self.joy_left_y = float(a(1))
        self.joy_right_x = float(a(2))
        self.joy_right_y = float(a(3))

        self.lin = self.apply_deadzone(self.joy_left_y)
        self.ang = self.apply_deadzone(self.joy_left_x)

        raw_L2 = a(4, 1.0)
        raw_R2 = a(5, 1.0)
        self.L2 = (raw_L2 + 1.0) / 2.0 if -1.0 <= raw_L2 <= 1.0 else float(raw_L2)
        self.R2 = (raw_R2 + 1.0) / 2.0 if -1.0 <= raw_R2 <= 1.0 else float(raw_R2)

        self.d_pad_x = int(a(6, 0))
        self.d_pad_y = int(a(7, 0))

        self.Square_btn = b(3)
        self.X_btn = b(0)
        self.O_btn = b(1)
        self.Triangle_btn = b(2)

        self.L1_btn = b(4)
        self.R1_btn = b(5)
        self.L2_btn = b(6)
        self.R2_btn = b(7)

        up_b = b(13)
        right_b = b(14)
        down_b = b(15)
        left_b = b(16)
        if up_b or right_b or down_b or left_b:
            self.up_btn = bool(up_b)
            self.down_btn = bool(down_b)
            self.left_btn = bool(left_b)
            self.right_btn = bool(right_b)
        else:
            self.up_btn = (self.d_pad_y > 0)
            self.down_btn = (self.d_pad_y < 0)
            self.left_btn = (self.d_pad_x < 0)
            self.right_btn = (self.d_pad_x > 0)

    def joy_callback(self, msg):
        self.process_joy(list(msg.axes), list(msg.buttons))

    def check_timeout(self):
        if time.monotonic() - self.last_joy_time > self.timeout:
            self.joystick_lost = True
        else:
            self.joystick_lost = False

    def as_dict(self):
        return {
            'up_btn': bool(self.up_btn),
            'down_btn': bool(self.down_btn),
            'left_btn': bool(self.left_btn),
            'right_btn': bool(self.right_btn),
            'Square_btn': bool(self.Square_btn),
            'X_btn': bool(self.X_btn),
            'O_btn': bool(self.O_btn),
            'Triangle_btn': bool(self.Triangle_btn),
            'L1_btn': bool(self.L1_btn),
            'R1_btn': bool(self.R1_btn),
            'L2_btn': bool(self.L2_btn),
            'R2_btn': bool(self.R2_btn),
            'L2': self.L2,
            'R2': self.R2,
            'd_pad_x': self.d_pad_x,
            'd_pad_y': self.d_pad_y,
            'joy_left_x': self.joy_left_x,
            'joy_left_y': self.joy_left_y,
            'joy_right_x': self.joy_right_x,
            'joy_right_y': self.joy_right_y,
        }

    def __repr__(self):
        return f"<PS4Controller {self.as_dict()}>"