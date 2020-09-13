cmake_minimum_required(VERSION 2.8.3)
project(control_line_world)



find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs prius_msgs)

catkin_package()

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)



add_executable(control_line_world_node src/control_line_world_node.cpp src/control_line_func.cpp)

target_link_libraries(control_line_world_node ${catkin_LIBRARIES})

