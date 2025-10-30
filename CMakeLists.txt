cmake_minimum_required(VERSION 3.8)
project(cbf_pc_selector)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(pc_selector_node
    src/CbfPcSelectorNode.cpp
    src/Sensor.cpp
)

target_include_directories(pc_selector_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(pc_selector_node
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
)

install(
    TARGETS pc_selector_node
    DESTINATION lib/${PROJECT_NAME}
)
install(
    DIRECTORY include
    DESTINATION include
)
install(
    DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
