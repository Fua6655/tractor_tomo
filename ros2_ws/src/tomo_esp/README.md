# tomo_esp

Version: 1.3 (final for phase 1)

micro-ROS agent (Docker) + ESP32 firmware (Arduino) that directly drives GPIO.

## Contents
- `scripts/start_micro_ros_agent.sh` - start micro-ROS agent in Docker
- `launch/micro_ros_agent.launch.py` - ROS 2 launch wrapper for agent
- `tomo_esp/esp_microros/esp_microros.ino` - ESP32 firmware

## Topic map (ESP32 firmware)
Subscribers:
- `/tomo/states` (tomo_msgs/OutputStates)
- `/tomo/engine_cmd` (std_msgs/Float32)
- `/tomo/steer_cmd` (std_msgs/Float32)

Publishers:
- `/tomo/esp_alive` (std_msgs/Bool)

## Hardware pinout (from `esp_microros.ino`)
Digital outputs:
- `ENGINE_START_PIN = 4`
- `CLUTCH_PIN = 5`
- `BRAKE_PIN = 6`
- `FRONT_POSITION_PIN = 7`
- `FRONT_SHORT_PIN = 8`
- `FRONT_LONG_PIN = 9`
- `BACK_POSITION_PIN = 10`
- `LEFT_BLINK_PIN = 11`
- `RIGHT_BLINK_PIN = 12`

PWM + direction:
- Throttle: `THR_EN = 13` (PWM), `THR_IN1 = 14`, `THR_IN2 = 15`
- Steering: `STR_EN = 16` (PWM), `STR_IN1 = 17`, `STR_IN2 = 18`

PWM config:
- `PWM_FREQ = 1000` Hz
- `PWM_RES_BITS = 8`
- Channels: `THR_CH = 0`, `STR_CH = 1`

## Launch
- `ros2 launch tomo_esp micro_ros_agent.launch.py port:=8888`

## Notes
- Agent runs `microros/micro-ros-agent:iron` in Docker.
- ESP32 uses WiFi transport and configuration from `esp_microros.ino`.
