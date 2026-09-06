cmake_minimum_required(VERSION 3.0.2)
project(diff_drive_gvf_sim)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf
)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(diff_drive_sim src/diff_drive_sim.cpp)
target_link_libraries(diff_drive_sim ${catkin_LIBRARIES})

add_executable(car_model_visualization src/car_model_visualization.cpp)
target_link_libraries(car_model_visualization ${catkin_LIBRARIES})
