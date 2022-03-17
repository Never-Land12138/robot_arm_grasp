cmake_minimum_required(VERSION 3.0.2)
project(main)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  geometry_msgs
)

catkin_package(INCLUDE_DIRS include CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(main src/main.cpp)

target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
)