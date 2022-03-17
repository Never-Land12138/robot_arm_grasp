add_executable(move_group_interface_tutorial src/move_group_interface_tutorial.cpp)
target_link_libraries(move_group_interface_tutorial ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS move_group_interface_tutorial DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

add_executable(robot_arm_server src/robot_arm_server.cpp)
target_link_libraries(robot_arm_server ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS robot_arm_server DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})