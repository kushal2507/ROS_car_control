cmake_minimum_required(VERSION 2.8.3)
project(opencv_line_detector)

find_package(catkin REQUIRED geometry_msgs prius_msgs sensor_msgs roscpp std_msgs vision_msgs cv_bridge image_transport )
find_package(OpenCV REQUIRED)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})


add_executable(opencv_line_detector_node src/opencv_line_detector_node.cpp)

target_link_libraries(opencv_line_detector_node ${catkin_LIBRARIES})
target_link_libraries(opencv_line_detector_node ${OpenCV_LIBRARIES})
