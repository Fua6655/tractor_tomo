# tomo_web

Version: 1.3 (final for phase 1)

Web UI + ROS bridge (FastAPI + WebSocket). Sends `ControlEvents` and displays system state in real time.

## Nodes
- `web_node`
  - Pub: `/control/events` (tomo_msgs/ControlEvents)
  - Pub: `/control/emergency` (tomo_msgs/Emergency)
  - Sub: `/tomo/states` (tomo_msgs/OutputStates)
  - Sub: `/tomo/engine_cmd` (std_msgs/Float32)
  - Sub: `/tomo/steer_cmd` (std_msgs/Float32)
  - Sub: `/tomo/esp_alive` (std_msgs/Bool)

## HTTP/WS
- HTTP server: `http://localhost:8000/`
- WebSocket: `/ws`
- Static UI: `tomo_web/html/index.html`

## Parameters (selected)
- `control_event_topic`, `control_emergency_topic`
- `output_topic`, `engine_cmd_topic`, `steer_cmd_topic`
- `esp_alive_topic`, `esp_alive_timeout`

## Launch
- `ros2 launch tomo_web web.launch.py`

## Notes
- `web_node` starts uvicorn on port `8000`.
- Failsafe indicator is derived from `/tomo/esp_alive` watchdog.
