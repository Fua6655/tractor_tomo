#!/usr/bin/env bash
set -euo pipefail

PORT="${1:-8888}"

exec docker run --rm --net=host \
  microros/micro-ros-agent:iron \
  udp4 --port "${PORT}" -v6
