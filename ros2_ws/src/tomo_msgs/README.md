# tomo_msgs

Version: 1.3 (final for phase 1)

Custom ROS 2 messages for the TOMO system.

## Messages
- `ControlEvents.msg`
  - Source: PS4 / WEB / AUTO
  - Categories: STATE / EVENT / SIGNALIZATION / SYSTEM
  - Type: specific action (arm, engine, blink/horn, force source, ...)
- `OutputStates.msg`
  - Aggregated system state (armed, engine, signalization, outputs)
- `Emergency.msg`
  - `active`, `level`, `reason`

## Package
- Build: standard ROS 2 msg package (`CMakeLists.txt`)
