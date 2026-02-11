micro-ROS Custom Messages (Arduino)
===================================

This folder keeps the local instructions and helper script to generate an
Arduino micro-ROS library that includes the custom `tomo_msgs` messages.

Why this is needed
------------------
The Arduino micro-ROS library only ships with standard message types.
To use `tomo_msgs/msg/OutputStates` on the ESP32, you must regenerate the
micro-ROS Arduino library with `tomo_msgs` included.

Prerequisites
-------------
- `micro_ros_arduino` repository cloned locally.
- Docker (used by the micro-ROS static library builder).

Recommended workflow
--------------------
1) Export the path to the `micro_ros_arduino` repo:

   export MICRO_ROS_ARDUINO_DIR=$HOME/Arduino/libraries/micro_ros_arduino

2) Run the helper script from this folder:

   ./generate_arduino_lib.sh

3) Optional: pick a builder tag (default is `iron`):

   export MICROROS_DOCKER_TAG=iron

Notes
-----
- The script regenerates a micro-ROS Arduino library that includes
  `tomo_msgs`. You then use that regenerated library in Arduino IDE.
- If your `micro_ros_arduino` path is different, set
  `MICRO_ROS_ARDUINO_DIR` before running.
