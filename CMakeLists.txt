cmake_minimum_required(VERSION 3.8)
project(laser_gazebo_resources)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS TRUE CACHE INTERNAL "")

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(laser_rahcm_simulation REQUIRED)
find_package(laser_usv_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(MAVLink REQUIRED)
find_package(Protobuf REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CustomMsg.msg"
  "msg/CustomPoint.msg"
  DEPENDENCIES std_msgs
)

set(PROTOBUF_IMPORT_DIRS "")
set(GAZEBO_MSG_INCLUDE_DIRS "")

foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(EXISTS "${ITR}/gazebo/msgs/proto")
    set(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
    set(GAZEBO_MSG_INCLUDE_DIRS "${ITR}/gazebo/msgs")
    break()
  endif()
endforeach()

if(NOT PROTOBUF_IMPORT_DIRS)
  set(PROTOBUF_IMPORT_DIRS "/usr/include/gazebo-11/gazebo/msgs/proto")
  set(GAZEBO_MSG_INCLUDE_DIRS "/usr/include/gazebo-11/gazebo/msgs")
endif()

message(STATUS "Gazebo Proto Path: ${PROTOBUF_IMPORT_DIRS}")

pkg_check_modules(PROTOBUF protobuf)

include_directories(
  include
  ${Boost_INCLUDE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${GAZEBO_INCLUDE_DIRS}
  ${GAZEBO_MSG_INCLUDE_DIRS}
  ${GAZEBO_LIBRARY_DIRS}
  ${MAVLINK_INCLUDE_DIRS}
  ${MAVLINK_INCLUDE_DIRS}/mavlink/v2.0 
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

set(proto_msg_files
  proto_msg/Airspeed.proto
  proto_msg/OpticalFlow.proto
  proto_msg/Odometry.proto
  proto_msg/Range.proto
  proto_msg/SITLGps.proto
  proto_msg/IRLock.proto
  proto_msg/TargetRelative.proto
  proto_msg/Groundtruth.proto
  proto_msg/MagneticField.proto
  proto_msg/Pressure.proto
  proto_msg/Imu.proto
  proto_msg/CommandMotorSpeed.proto
  proto_msg/Float.proto
  proto_msg/MotorSpeed.proto
  proto_msg/Wind.proto
  proto_msg/Int32.proto
)

PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${proto_msg_files})
add_library(proto_msg SHARED ${PROTO_SRCS})

add_library(link_static_tf_publisher SHARED
    plugins/link_static_tf_publisher.cpp
)
ament_target_dependencies(link_static_tf_publisher rclcpp tf2_ros geometry_msgs)
target_include_directories(link_static_tf_publisher PUBLIC
  ${GAZEBO_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(link_static_tf_publisher ${GAZEBO_LIBRARIES})

add_library(LivoxPluginGazebo SHARED
    src/livox_points_plugin.cpp
    src/livox_ode_multiray_shape.cpp
)
ament_target_dependencies(LivoxPluginGazebo
    rclcpp gazebo_ros sensor_msgs pcl_conversions tf2 tf2_ros tf2_msgs ament_index_cpp
)
target_include_directories(LivoxPluginGazebo PUBLIC
    ${GAZEBO_INCLUDE_DIRS}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(LivoxPluginGazebo
    ${cpp_typesupport_target}
    ${GAZEBO_LIBRARIES}
)

add_executable(csv_main src/csv_main.cpp)
ament_target_dependencies(csv_main rclcpp)
target_include_directories(csv_main PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

add_library(MotorPluginGazebo SHARED
    src/gazebo_motor_model.cpp
)
target_include_directories(MotorPluginGazebo PUBLIC
    ${GAZEBO_INCLUDE_DIRS}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(MotorPluginGazebo
    ${Boost_LIBRARIES}
    ${GAZEBO_LIBRARIES}
    proto_msg
)

add_library(MavlinkPluginGazebo SHARED
    src/gazebo_mavlink_interface.cpp src/mavlink_interface.cpp
)
target_include_directories(MavlinkPluginGazebo SYSTEM PUBLIC
    ${GAZEBO_INCLUDE_DIRS}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(MavlinkPluginGazebo
  rclcpp
	std_msgs
	laser_rahcm_simulation
	laser_usv_msgs
)
target_link_libraries(MavlinkPluginGazebo
    ${Boost_LIBRARIES}
    ${GAZEBO_LIBRARIES}
    proto_msg
)

install(TARGETS
  proto_msg
  link_static_tf_publisher
  LivoxPluginGazebo
  MotorPluginGazebo
  MavlinkPluginGazebo
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS csv_main
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY models worlds resources DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/laser_gazebo_resources.dsv.in")

ament_export_dependencies(
  rosidl_default_runtime rclcpp pcl_conversions tf2 tf2_ros tf2_msgs
  geometry_msgs std_msgs sensor_msgs gazebo_ros rclcpp laser_rahcm_simulation
)

ament_package()
