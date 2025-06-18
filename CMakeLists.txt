cmake_minimum_required(VERSION 3.0.2)
project(cbf_pc_selector)

set(CMAKE_CXX_FLAGS "-O3")
add_compile_options(-std=c++17)

find_package(yaml-cpp REQUIRED)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    tf2_ros
    tf2_sensor_msgs
)

catkin_package(
    LIBRARIES cbf_pc_selector
    CATKIN_DEPENDS roscpp sensor_msgs tf2_ros tf2_sensor_msgs
)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${catkin_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIR}
)

add_executable(cbf_pc_selector_node src/CbfPcSelectorNode.cpp src/Sensor.cpp)
target_link_libraries(
    cbf_pc_selector_node
    ${catkin_LIBRARIES}
    yaml-cpp
)
