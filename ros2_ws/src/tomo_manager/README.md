# tomo_manager

Version: 1.3 (final for phase 1)

Low-level managers that convert `/tomo/states` and `/tomo/cmd_vel` into actuator commands.

## Nodes
- `engine_manager`
  - Sub: `/tomo/states` (tomo_msgs/OutputStates)
  - Sub: `/tomo/cmd_vel` (geometry_msgs/Twist)
  - Pub: `/tomo/engine_cmd` (std_msgs/Float32)
- `steering_manager`
  - Sub: `/tomo/cmd_vel` (geometry_msgs/Twist)
  - Sub: `/control/emergency` (tomo_msgs/Emergency)
  - Pub: `/tomo/steer_cmd` (std_msgs/Float32)

## Parameters (selected)
`engine_manager`:
- `idle_throttle` (default 0.15)
- `start_throttle` (default 0.9)

`steering_manager`:
- `max_angle_deg`, `soft_limit_margin_deg`
- `deadzone`, `steer_scale`
- `control_rate`, `cmd_timeout`, `max_steer_rate`

## Launch
- `ros2 launch tomo_manager engine.launch.py`
- `ros2 launch tomo_manager steering.launch.py`
