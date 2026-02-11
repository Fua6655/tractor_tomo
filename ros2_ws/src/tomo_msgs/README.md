# tomo_msgs

Version: 1.3 (final for phase 1)

Custom ROS 2 messages for the TOMO system.

## Messages
- `ControlEvents.msg`
  - Source: PS4 / WEB / AUTO
  - Categories: STATE / EVENT / LIGHT / SYSTEM
  - Type: specific action (arm, power, blink, force source, ...)
- `OutputStates.msg`
  - Aggregated system state (armed, power, light, outputs)
- `Emergency.msg`
  - `active`, `level`, `reason`

## Package
- Build: standard ROS 2 msg package (`CMakeLists.txt`)
