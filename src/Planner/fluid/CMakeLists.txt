cmake_minimum_required(VERSION 3.0.2)
project(fluid)
set(CMAKE_CXX_STANDARD 14)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -g -Wall")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  geometry_msgs
  std_msgs
  visualization_msgs
  message_generation
  message_filters
  cv_bridge
  common_msgs
  plan_env
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES fluid
  CATKIN_DEPENDS roscpp rospy tf geometry_msgs std_msgs visualization_msgs message_runtime common_msgs plan_env
  DEPENDS message_generation
)
include_directories(
  SYSTEM
  include 
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}/devel/include
)

add_library(fluid_gvf_planner
   src/gvf_manager.cpp
   src/gvf.cpp
   src/fluid_guidance.cpp
   src/fluid_solver_2d.cpp
   src/fluid_solver_3d.cpp
   src/flow_coordinates_3d.cpp
)
target_link_libraries(fluid_gvf_planner ${catkin_LIBRARIES})

# updateESDF3d() parallelizes its distance-transform scanlines with OpenMP;
# without OpenMP the pragmas are ignored and the code runs serially.
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
  target_link_libraries(fluid_gvf_planner OpenMP::OpenMP_CXX)
endif()


add_executable( formation_planning 
    src/formation_planning.cpp
  )
target_link_libraries(formation_planning 
fluid_gvf_planner
 ${catkin_LIBRARIES}
)


if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(flow_coordinates_3d_test test/flow_coordinates_3d_test.cpp)
  if(TARGET flow_coordinates_3d_test)
    target_link_libraries(flow_coordinates_3d_test fluid_gvf_planner ${catkin_LIBRARIES} gtest_main)
  endif()
endif()
