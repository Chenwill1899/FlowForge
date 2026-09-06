cmake_minimum_required(VERSION 3.0.2)
project(gvf_cmd_bridge)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  quadrotor_msgs
  tf
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(position_cmd_to_twist_bridge src/position_cmd_to_twist_bridge.cpp)
target_link_libraries(position_cmd_to_twist_bridge ${catkin_LIBRARIES})
add_dependencies(position_cmd_to_twist_bridge ${catkin_EXPORTED_TARGETS})
