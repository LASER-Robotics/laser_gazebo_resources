cmake_minimum_required(VERSION 3.8)
project(laser_gazebo_resources)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gazebo REQUIRED)

add_library(link_static_tf_publisher SHARED
    plugins/link_static_tf_publisher.cpp
    )

ament_target_dependencies(link_static_tf_publisher
  rclcpp
  tf2_ros
  geometry_msgs
)

target_include_directories(link_static_tf_publisher PUBLIC
  ${GAZEBO_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(link_static_tf_publisher
  ${GAZEBO_LIBRARIES}
)


install(TARGETS link_static_tf_publisher
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY
  models
  worlds
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/laser_gazebo_resources.dsv.in")

ament_package()
