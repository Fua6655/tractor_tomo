cmake_minimum_required(VERSION 3.8)
project(tomo_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ControlEvents.msg"
  "msg/OutputStates.msg"
  "msg/Emergency.msg"
  DEPENDENCIES geometry_msgs
)

ament_package()

