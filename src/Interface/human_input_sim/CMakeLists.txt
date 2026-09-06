cmake_minimum_required(VERSION 3.0.2)
project(human_input_sim)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs rospy sensor_msgs
)

catkin_install_python(PROGRAMS
  scripts/joy_to_intent.py
  scripts/joy_trace_replay.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
