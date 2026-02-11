# TOMO Control System — Project Documentation

This repository contains a ROS 2 workspace (`ros2_ws`) for a tractor/vehicle control system.
The core is an event-based state machine (ControlFactory) that converts input events
(PS4/Web/Auto) into output states and commands for ESP32.

---

## Workspace Layout

- `ros2_ws/` — ROS 2 workspace
  - `src/tomo_factory/` — ControlFactory + MotionFactory (core logic)
  - `src/tomo_manager/` — EngineManager + SteeringManager (low-level outputs)
  - `src/tomo_ps4/` — PS4 input node (events + cmd_vel)
  - `src/tomo_web/` — Web UI + ROS bridge (FastAPI + WebSocket)
  - `src/tomo_esp/` — micro-ROS agent (Docker) + ESP32 firmware
  - `src/tomo_msgs/` — custom ROS 2 message definitions
  - `src/tomo_bringup/` — launch files
  - `src/tomo_auto/` — placeholder for autonomy (development just started)
- `images/` — system images/diagrams
- `docs/` — project documentation

---

## Versions

- **v1.3**: final version for phase 1
  - `tomo_factory`, `tomo_manager`, `tomo_ps4`, `tomo_web`, `tomo_esp`, `tomo_msgs`, `tomo_bringup`
- **tomo_auto**: not part of v1.3, development just started

---

## Packages (short)

### `tomo_factory`
- Reducer + FSM (armed/power/light, emergency)
- Publishes `OutputStates`
- Motion mux + accel/jerk limiting

### `tomo_manager`
- `engine_manager` -> `/tomo/engine_cmd`
- `steering_manager` -> `/tomo/steer_cmd`

### `tomo_ps4`
- PS4 -> `ControlEvents` + `/ps4/cmd_vel`
- Timeout => HARD emergency

### `tomo_web`
- FastAPI server + WebSocket
- Sends `ControlEvents` + displays state

### `tomo_esp`
- Docker micro-ROS agent
- ESP32 firmware (`esp_microros.ino`) drives GPIO

### `tomo_msgs`
- `ControlEvents`, `OutputStates`, `Emergency`

### `tomo_bringup`
- Launch files for full or partial bringup

---

## ROS Topics (core)

Input:
- `/control/events` (tomo_msgs/ControlEvents)
- `/control/emergency` (tomo_msgs/Emergency)
- `/ps4/cmd_vel` (geometry_msgs/Twist)
- `/auto/cmd_vel` (geometry_msgs/Twist)
- `/joy` (sensor_msgs/Joy)

Outputs:
- `/tomo/states` (tomo_msgs/OutputStates)
- `/tomo/cmd_vel` (geometry_msgs/Twist)
- `/tomo/engine_cmd` (std_msgs/Float32)
- `/tomo/steer_cmd` (std_msgs/Float32)
- `/tomo/esp_alive` (std_msgs/Bool)

---

## Build and Run

From repo root:

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

Full bringup (PS4 + web + control + motion + managers):

```bash
ros2 launch tomo_bringup tomo_system.launch.py
```

Individual launches:

```bash
ros2 launch tomo_factory control.launch.py
ros2 launch tomo_factory motion.launch.py
ros2 launch tomo_ps4 ps4.launch.py
ros2 launch tomo_web web.launch.py
ros2 launch tomo_manager engine.launch.py
ros2 launch tomo_manager steering.launch.py
ros2 launch tomo_esp micro_ros_agent.launch.py
```

Web UI:

```bash
ros2 run tomo_web web_node
# open in browser: http://localhost:8000/
```

---

## micro-ROS agent (Docker)

```bash
ros2 launch tomo_esp micro_ros_agent.launch.py port:=8888
```

- Runs `microros/micro-ros-agent:iron` in Docker.
- ESP32 firmware uses WiFi transport and configuration from `esp_microros.ino`.

---

## More detail

- Each package has its own `README.md` inside its folder.
