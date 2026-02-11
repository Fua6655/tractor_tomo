# tomo_ps4

Version: 1.3 (final for phase 1)

PS4 input node. Converts joystick input into `ControlEvents` + `cmd_vel` and triggers emergency on timeout.

## Nodes
- `ps4_node`
  - Sub: `/joy` (sensor_msgs/Joy)
  - Pub: `/control/events` (tomo_msgs/ControlEvents)
  - Pub: `/control/emergency` (tomo_msgs/Emergency)
  - Pub: `/ps4/cmd_vel` (geometry_msgs/Twist)

## PS4 mapping (current)
Holding buttons sends state toggles; taps/edges send events. Mapping matches `ps4_node.py`.

- **X (hold)**: ARM toggle
- **O (hold)**: POWER toggle
- **Square (hold)**: LIGHT mode toggle
- **Triangle (hold)**: ENGINE_START (value = 1 while pressed)
- **L1**: CLUTCH active (event category)
- **R1**: BRAKE active (event category)
- **L1 (hold)**: MOVE_ALLOWED toggle while held for `move_hold_time`
- **D-pad** (only when LIGHT mode is ON):
  - Up: FRONT_SEQUENCE_NEXT
  - Down: BACK_POSITION
  - Left: LEFT_BLINK
  - Right: RIGHT_BLINK

## Parameters (selected)
- `arm_hold_time`, `power_hold_time`, `light_hold_time`, `move_hold_time`
- `joy_topic`, `control_event_topic`, `control_emergency_topic`, `cmd_topic`

## Launch
- `ros2 launch tomo_ps4 ps4.launch.py`

## Notes
- `ps4.launch.py` also starts `joy_node` from the `joy` package.
- Joystick timeout sends HARD emergency (`reason=ps4_timeout`).
