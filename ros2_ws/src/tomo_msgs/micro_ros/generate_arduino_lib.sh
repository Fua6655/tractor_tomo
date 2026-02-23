#!/usr/bin/env bash
set -euo pipefail

if [[ -z "${MICRO_ROS_ARDUINO_DIR:-}" ]]; then
  echo "MICRO_ROS_ARDUINO_DIR is not set."
  echo "Example: export MICRO_ROS_ARDUINO_DIR=\$HOME/Arduino/libraries/micro_ros_arduino"
  exit 1
fi

GEN_DIR="$MICRO_ROS_ARDUINO_DIR/extras/library_generation"

if [[ ! -f "$GEN_DIR/library_generation.sh" ]]; then
  echo "library_generation.sh not found at:"
  echo "  $GEN_DIR/library_generation.sh"
  echo "Check MICRO_ROS_ARDUINO_DIR and your micro_ros_arduino checkout."
  exit 1
fi

if ! command -v docker >/dev/null 2>&1; then
  echo "docker is required for library generation."
  exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

EXTRA_PKGS="$GEN_DIR/extra_packages"
mkdir -p "$EXTRA_PKGS"

rm -rf "$EXTRA_PKGS/tomo_msgs"
cp -R "$PKG_DIR" "$EXTRA_PKGS/tomo_msgs"

DOCKER_TAG="${MICROROS_DOCKER_TAG:-iron}"

echo "Generating micro-ROS Arduino library with tomo_msgs (docker tag: $DOCKER_TAG)..."
docker pull "microros/micro_ros_static_library_builder:${DOCKER_TAG}"
docker run -it --rm \
  -v "$MICRO_ROS_ARDUINO_DIR":/project \
  --env MICROROS_LIBRARY_FOLDER=extras \
  "microros/micro_ros_static_library_builder:${DOCKER_TAG}"

# Some Arduino ESP32-S3 toolchains look for src/esp32s3 while the generated
# micro-ROS library provides src/esp32. Create a compatibility alias.
if [[ -d "$MICRO_ROS_ARDUINO_DIR/src/esp32" && ! -e "$MICRO_ROS_ARDUINO_DIR/src/esp32s3" ]]; then
  ln -s esp32 "$MICRO_ROS_ARDUINO_DIR/src/esp32s3"
  echo "Created compatibility symlink: src/esp32s3 -> src/esp32"
fi

echo "Done. Restart Arduino IDE after generation."
