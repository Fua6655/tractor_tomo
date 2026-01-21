#!/usr/bin/env python3

from enum import Enum
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray, String, Bool
from geometry_msgs.msg import Twist
from tomo_msgs import Emergency

class ControlState(Enum):
    JOYSTICK = 1
    WEB = 2
    AUTO = 3
    EMERGENCY = 4
