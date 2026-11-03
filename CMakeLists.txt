cmake_minimum_required(VERSION 3.8)
project(arm_6dof_description)

find_package(ament_cmake REQUIRED)

install(
  PROGRAMS scripts/pick_and_place_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY urdf config launch worlds scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
