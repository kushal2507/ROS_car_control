cmake_minimum_required(VERSION 2.8.3)

#Name of the package
project(pcl_obstacle_detector)

#find the catkin build system and any other packages on which it depends
find_package(catkin REQUIRED COMPONENTS roscpp pcl_ros pcl_conversions)

# declare this catkin_package
catkin_package()

#specify location of header files
include_directories(${catkin_INCLUDE_DIRS})

#declare executables along with source files
add_executable(pcl_obstacle_detector_node src/pcl_obstacle_detector_node.cpp)

#Specify libraries against which to link.
target_link_libraries(pcl_obstacle_detector_node ${catkin_LIBRARIES})
