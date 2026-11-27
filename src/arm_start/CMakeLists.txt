cmake_minimum_required(VERSION 3.8)
project(arm_start)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/config")
  install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
endif()

install(
  PROGRAMS scripts/attach_board.sh scripts/attach_camera.sh scripts/camera_driver.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
