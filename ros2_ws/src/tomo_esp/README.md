# tomo_esp

Version: 1.3 (final for phase 1)

micro-ROS agent (Docker) + ESP32 firmware (Arduino) that directly drives GPIO.

## Contents
- `scripts/start_micro_ros_agent.sh` - start micro-ROS agent in Docker
- `launch/micro_ros_agent.launch.py` - ROS 2 launch wrapper for agent
- `tomo_esp/microros_1/microros_1.ino` - micro-ROS ESP #1 (states I/O)
- `tomo_esp/microros_2/microros_2.ino` - micro-ROS ESP #2 (throttle/steering)

## Topic map (split micro-ROS)
ESP #1 (`microros_1`):
- Sub: `/tomo/states` (tomo_msgs/OutputStates)

ESP #2 (`microros_2`):
- Sub: `/tomo/engine_cmd` (std_msgs/Float32)
- Sub: `/tomo/steer_cmd` (std_msgs/Float32)

Alive publishers:
- `microros_1` -> `/tomo/esp1_alive` (std_msgs/Bool)
- `microros_2` -> `/tomo/esp2_alive` (std_msgs/Bool)

## Hardware pinout (`microros_1`)
Digital outputs:
- `ENGINE_START_PIN = 36`
- `CLUTCH_PIN = 37`
- `BRAKE_PIN = 38`
- `FRONT_POSITION_PIN = 9`
- `FRONT_SHORT_PIN = 10`
- `FRONT_LONG_PIN = 11`
- `BACK_POSITION_PIN = 15`
- `LEFT_BLINK_PIN = 16`
- `RIGHT_BLINK_PIN = 17`
- `HORN_PIN = 18`
- `ARMED_PIN = 5`
- `ENGINE_ON_PIN = 6`
- `MOVE_ALLOWED_PIN = 7`

Behavior:
- `HORN_PIN` mirrors `OutputStates.horn`

## Hardware pinout (`microros_2`)
PWM + direction:
- Throttle: `THR_EN = 5` (PWM), `THR_IN1 = 6`, `THR_IN2 = 7`
- Steering: `STR_EN = 12` (PWM), `STR_IN1 = 13`, `STR_IN2 = 14`

PWM config:
- `PWM_FREQ = 1000` Hz
- `PWM_RES_BITS = 8`
- Channels: `THR_CH = 0`, `STR_CH = 1`

## Launch
- `ros2 launch tomo_esp micro_ros_agent.launch.py port:=8888`

## Notes
- Agent runs `microros/micro-ros-agent:iron` in Docker.
- Both ESP boards use WiFi transport and connect directly to the same micro-ROS agent.

## Power manager firmware (no micro-ROS)
File:
- `tomo_esp/power_manager/power_manager.ino`

I/O:
- Inputs: `POWER_BUTTON_PIN = 32`, `ESTOP_PIN = 33` (active-low, `INPUT_PULLUP`)
- Outputs: `LED1_PIN = 25`, `LED2_PIN = 26`
- Relay outputs:
  - `RELAY_CH1_PS_ON = 18`
  - `RELAY_CH2_MICROROS_PWR = 19`
  - `RELAY_CH3_FAN = 21`
  - `RELAY_CH4_POWER_OUTPUTS = 22` (uses former CH4 channel for dedicated micro-ROS ESP power relay)

UDP:
- ESP listen port: `5005`
- Raspberry -> ESP commands:
  - `ROS_READY`
  - `AGENT_READY`
  - `AGENT_STOPPED`
  - `ROS_STOPPED`
  - `LAST_MESSAGE`
  - `SAFE_TO_POWER_OFF`
  - `HEARTBEAT`
  - JSON heartbeat example:
    - `{"ros": true, "agent": true, "load": 0.35, "temp": 48}`
- ESP -> Raspberry messages:
  - `PING`
  - `SHUTDOWN`
  - `ACK_AGENT_STOPPED`
  - `ACK_ROS_STOPPED`
  - `ACK_LAST_MESSAGE`
  - `ACK_SAFE_TO_POWER_OFF`
  - JSON status frame (periodic)

Power manager FSM states:
- `OFF`
- `POWERING_ON`
- `WAITING_RPI_BOOT`
- `WAITING_ROS_READY`
- `WAITING_AGENT_READY`
- `RUNNING`
- `SHUTDOWN_REQUESTED`
- `WAITING_SAFE_SHUTDOWN`
- `ERROR_STATE`

Emergency behavior:
- On E-stop press, `RELAY_CH4_POWER_OUTPUTS` is switched OFF immediately.
- Power manager ESP remains active (still powered from 5VSB) and continues UDP/FSM handling.
