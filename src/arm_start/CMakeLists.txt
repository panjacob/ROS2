cmake_minimum_required(VERSION 3.8)
project(arm_start)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/attach_board.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
