cmake_minimum_required(VERSION 3.8)
project(laser_gazebo_resources)

# --- 1. Padrão C++ e Opções de Compilação ---
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- 2. Encontrar Dependências ---
find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# --- 3. Gerar Mensagens e Serviços ---
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CustomMsg.msg"
  "msg/CustomPoint.msg"
  DEPENDENCIES std_msgs
)

set(PROTOBUF_IMPORT_DIRS "")
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
  endif()
endforeach()

find_package(Protobuf REQUIRED)
pkg_check_modules(PROTOBUF protobuf)

set(GAZEBO_MSG_INCLUDE_DIRS)
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(GAZEBO_MSG_INCLUDE_DIRS "${ITR}/gazebo/msgs")
  endif()
endforeach()

# --- 4. Definir Bibliotecas, Executáveis e Dependências ---
include_directories(
  include
  ${Boost_INCLUDE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${GAZEBO_INCLUDE_DIRS}
  ${GAZEBO_MSG_INCLUDE_DIRS}
  ${GAZEBO_LIBRARY_DIRS}
  )

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# -- Plugin: link_static_tf_publisher --
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

# -- Plugin: livox_points_plugin --
# CORREÇÃO 1: Renomeado o alvo para corresponder ao nome no arquivo de mundo
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

set(proto_msg
  proto_msg/CommandMotorSpeed.proto
  proto_msg/Float.proto
  proto_msg/MotorSpeed.proto
  proto_msg/Wind.proto
)

PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${proto_msg})
add_library(proto_msg SHARED ${PROTO_SRCS})

# Adicionar build do motor model
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

# --- 5. Instalação ---
# CORREÇÃO 2: Simplificado o destino da instalação para 'lib'
install(TARGETS
  link_static_tf_publisher
  LivoxPluginGazebo
  MotorPluginGazebo
  csv_main
  DESTINATION lib
)

install(DIRECTORY models worlds resources DESTINATION share/${PROJECT_NAME})

# --- 6. Testes ---
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# --- 7. Hooks de Ambiente ---
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/laser_gazebo_resources.dsv.in")

# --- 8. Exportar Dependências e Registrar Pacote ---
ament_export_dependencies(
  rosidl_default_runtime rclcpp pcl_conversions tf2 tf2_ros tf2_msgs
  geometry_msgs std_msgs sensor_msgs
)

ament_package()
