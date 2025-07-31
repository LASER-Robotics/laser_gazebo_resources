cmake_minimum_required(VERSION 3.8)
project(laser_gazebo_resources)

# Find dependencies
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
  models
  worlds
  DESTINATION share/${PROJECT_NAME})

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/laser_gazebo_resources.dsv.in")

ament_package()
