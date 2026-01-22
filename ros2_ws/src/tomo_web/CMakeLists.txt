cmake_minimum_required(VERSION 3.8)
project(tomo_web)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  tomo_web/web_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
