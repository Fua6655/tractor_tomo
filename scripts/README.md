# Scripts

Helper scripts for building and running the TOMO ROS 2 workspace.

## build_tomo.sh
- Cleans `build/`, `install/`, `log/`
- Sources ROS 2 Jazzy
- Runs `colcon build`

```bash
./scripts/build_tomo.sh
```

## start_micro_ros_agent.sh
- Sources the local workspace
- Stops any process bound to UDP port 8888 (if `sudo` is available)
- Launches the micro-ROS agent via `tomo_esp`

```bash
./scripts/start_micro_ros_agent.sh
```

## stop_micro_ros_agent.sh
- Stops the Docker container running `microros/micro-ros-agent`

```bash
./scripts/stop_micro_ros_agent.sh
```

## tomo_launch.sh
- Sources the local workspace
- Launches the full system bringup

```bash
./scripts/tomo_launch.sh
```

## Requirements
- ROS 2 Jazzy installed at `/opt/ros/jazzy`
- `colcon` in PATH
- Docker (for micro-ROS agent)
