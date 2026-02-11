# TOMO Control System (ROS 2)

Event-based control system for a tractor/vehicle. Inputs (PS4/Web/Auto) feed a
central FSM, which produces output states and commands for ESP32.

![System Architecture](images/hardware_nodes.png)

---

## Quick start

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

Full bringup:

```bash
ros2 launch tomo_bringup tomo_system.launch.py
```

Web UI:

```bash
ros2 run tomo_web web_node
# browser: http://localhost:8000/
```

---

## Packages

- `tomo_factory` — FSM + event reducer + motion mux
- `tomo_manager` — engine/steering managers
- `tomo_ps4` — PS4 -> ControlEvents + cmd_vel
- `tomo_web` — Web UI + ROS bridge (FastAPI/WS)
- `tomo_esp` — micro-ROS agent (Docker) + ESP32 firmware
- `tomo_msgs` — custom messages
- `tomo_bringup` — launch files
- `tomo_auto` — in development (autonomy)

---

## Versions

- **v1.3**: final for phase 1 (all packages except `tomo_auto`)
- **tomo_auto**: development just started

---

## Documentation

- Project overview: `docs/PROJECT.md`
- Per-package docs: `ros2_ws/src/<package>/README.md`
