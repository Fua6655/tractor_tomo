# tomo_factory

Version: 1.3 (final for phase 1)

Central logic and safety layer. Contains two cores:
- **ControlFactory** (FSM + reducer) that turns `ControlEvents` into `OutputStates`
- **MotionFactory** that multiplexes `cmd_vel` and limits dynamics

## Nodes
- `control_factory`
  - Sub: `/control/events` (tomo_msgs/ControlEvents)
  - Sub: `/control/emergency` (tomo_msgs/Emergency)
  - Pub: `/tomo/states` (tomo_msgs/OutputStates)
- `motion_factory`
  - Sub: `/ps4/cmd_vel` (geometry_msgs/Twist)
  - Sub: `/auto/cmd_vel` (geometry_msgs/Twist)
  - Sub: `/control/emergency` (tomo_msgs/Emergency)
  - Sub: `/tomo/states` (tomo_msgs/OutputStates)
  - Pub: `/tomo/cmd_vel` (geometry_msgs/Twist)

## Parameters (selected)
`control_factory`:
- `control_event_topic` (default `/control/events`)
- `control_emergency_topic` (default `/control/emergency`)
- `output_topic` (default `/tomo/states`)

`motion_factory`:
- `ps4_cmd_topic`, `auto_cmd_topic`, `output_cmd_topic`, `states_topic`
- `control_emergency_topic`
- `failsafe_enabled`
- `control_rate`, `max_lin_accel`, `max_ang_accel`, `max_lin_jerk`, `max_ang_jerk`

## Launch
- `ros2 launch tomo_factory control.launch.py`
- `ros2 launch tomo_factory motion.launch.py`

## Notes
- Active source (PS4/WEB/AUTO) decides which `ControlEvents` are accepted.
- Emergency: HARD blocks all and forces blinkers; SOFT blocks dangerous events.
