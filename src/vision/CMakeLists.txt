cmake_minimum_required(VERSION 3.0.2)
project(vision)

set(OpenCV_DIR /usr/local/lib/cmake/opencv4/)

find_package(catkin REQUIRED COMPONENTS
  OpenCV REQUIRED
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
)

catkin_package(
  INCLUDE_DIRS include CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(calibrate src/calibrate.cpp)

target_link_libraries(calibrate ${catkin_LIBRARIES})