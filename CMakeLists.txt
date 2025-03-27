cmake_minimum_required(VERSION 3.0.2)
project(cbf_pc_selector)

set(CMAKE_CXX_FLAGS "-O3")
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    mavros_msgs
)

catkin_package(
    LIBRARIES cbf_pc_selector
    CATKIN_DEPENDS roscpp sensor_msgs mavros_msgs
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

add_executable(cbf_pc_selector_node src/cbf_pc_selector_node.cpp)
target_link_libraries(
    cbf_pc_selector_node
    ${catkin_LIBRARIES}
)
