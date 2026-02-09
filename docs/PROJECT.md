# TOMO Control System — Project Documentation

This repository contains a ROS 2 workspace (`ros2_ws`) for a tractor/vehicle control
system. The core design is a centralized event-based state machine (ControlFactory)
that turns input events (PS4/Web/Auto) into output states for the vehicle and ESP
firmware. The rest of the packages are sources, managers, and bridges.

---

## Workspace Layout

- `ros2_ws/` — ROS 2 workspace
  - `src/tomo_factory/` — ControlFactory + MotionFactory (core logic)
  - `src/tomo_manager/` — EngineManager + SteeringManager (low-level outputs)
  - `src/tomo_ps4/` — PS4 input node (events + cmd_vel)
  - `src/tomo_web/` — Web UI + ROS bridge (FastAPI + WebSocket)
  - `src/tomo_esp/` — UDP bridge to ESP firmware
  - `src/tomo_msgs/` — Custom ROS 2 message definitions
  - `src/tomo_bringup/` — Launch files (system bringup)
  - `src/tomo_auto/` — Placeholder package for autonomy input
- `images/` — System and UI screenshots

---

## Packages and Responsibilities

### `tomo_factory`
Central logic and safety layer.

- `control_factory.py`
  - Consumes `/control/events` and `/control/emergency`
  - Maintains FSM (arm, power, light)
  - Publishes `/tomo/states` (OutputStates)
  - Handles blink logic, emergency override, and source arbitration

- `motion_factory.py`
  - Muxes `/ps4/cmd_vel` and `/auto/cmd_vel`
  - Applies accel/jerk limits
  - Publishes `/tomo/cmd_vel`
  - Emergency handling: hard = instant zero, soft = ramp down

### `tomo_manager`
Low-level managers derived from `/tomo/cmd_vel` and `/tomo/states`.

- `engine_manager.py`
  - Publishes `/tomo/engine_cmd` (Float32 throttle 0..1)
  - Behavior depends on `engine_start`, `engine_stop`, `move_allowed`

- `steering_manager.py`
  - Publishes `/tomo/steer_cmd` (Float32 -1..1)
  - Deadzone, soft limits, max rate, timeout handling
  - Emergency -> steer to zero

### `tomo_ps4`
PS4 input handling and mapping to events.

- `ps4_node.py`
  - Reads `/joy` and publishes:
    - `/control/events` (ControlEvents)
    - `/control/emergency` (Emergency)
    - `/ps4/cmd_vel` (Twist)

### `tomo_web`
Web UI and ROS bridge.

- `web_node.py`
  - FastAPI server + WebSocket at `/ws`
  - Publishes `/control/events` and `/control/emergency`
  - Subscribes `/tomo/states`, `/tomo/engine_cmd`, `/tomo/steer_cmd`
  - UDP listener on 9999 for ESP telemetry/logs
  - Serves static UI from `tomo_web/html/index.html`

### `tomo_esp`
UDP bridge to ESP32 firmware.

- `esp_bridge.py`
  - Sends `OUT`, `THR`, `STR` payloads over UDP
  - Heartbeat + ACK latency tracking
  - Engine/steer watchdogs for timeout-to-zero

### `tomo_msgs`
Custom messages used across the system.

- `ControlEvents.msg`
  - Source: PS4 / WEB / AUTO
  - Category: STATE / EVENT / LIGHT / SYSTEM
  - Type: specific action (armed, engine_start, blinkers, etc)
  - Value: int (toggle/force)

- `OutputStates.msg`
  - Aggregated system state for outputs and UI

- `Emergency.msg`
  - `active`, `level`, `reason`

### `tomo_bringup`
Launch files for entire system or subsets.

---

## ROS Topics (Core)

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

Networking:
- UDP tx to ESP: `esp_ip:esp_port` (default 192.168.0.187:8888)
- UDP rx from ESP: port `esp_port + 1` (default 8889)
- Web UDP listener: port `9999` (for telemetry/logs)

---

## Build and Run

From repo root:

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

Launch full bringup (PS4 + web + control + motion + managers + ESP):

```bash
ros2 launch tomo_bringup bringup.launch.py
```

Or launch individual stacks:

```bash
ros2 launch tomo_factory control.launch.py
ros2 launch tomo_factory motion.launch.py
ros2 launch tomo_ps4 ps4.launch.py
ros2 launch tomo_esp esp.launch.py
ros2 launch tomo_web web.launch.py
ros2 launch tomo_manager engine.launch.py
ros2 launch tomo_manager steering.launch.py
```

Start the web UI service:

```bash
ros2 run tomo_web web_node
# then open in browser:
# http://localhost:8000/
```

---

## Parameters (Selected)

### ControlFactory
- `control_event_topic` (default `/control/events`)
- `control_emergency_topic` (default `/control/emergency`)
- `output_topic` (default `/tomo/states`)

### MotionFactory
- `ps4_cmd_topic`, `auto_cmd_topic`, `output_cmd_topic`, `states_topic`
- `failsafe_enabled` (bool)
- `control_rate`, `max_lin_accel`, `max_ang_accel`, `max_lin_jerk`, `max_ang_jerk`

### EngineManager
- `idle_throttle` (default 0.15)
- `start_throttle` (default 0.9)

### SteeringManager
- `max_angle_deg`, `soft_limit_margin_deg`, `deadzone`
- `cmd_timeout`, `max_steer_rate`, `control_rate`

### EspBridge
- `esp_ip`, `esp_port`
- `heartbeat_rate`, `engine_watchdog_rate`, `engine_timeout`
- `steer_watchdog_rate`, `steer_timeout`

---

## Data Flow (High Level)

1. PS4 / Web / Auto publish ControlEvents
2. ControlFactory reduces to OutputStates
3. MotionFactory muxes cmd_vel to `/tomo/cmd_vel`
4. EngineManager + SteeringManager convert to `/tomo/engine_cmd` + `/tomo/steer_cmd`
5. EspBridge sends UDP packets to ESP firmware

---

## Notes for Development

- `ros2_ws/build` and `ros2_ws/install` are generated artifacts.
- `tomo_auto` is currently a placeholder; you can add an autonomy node that publishes
  `ControlEvents` and `/auto/cmd_vel`.
