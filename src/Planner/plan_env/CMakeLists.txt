cmake_minimum_required(VERSION 2.8.3)
project(plan_env)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -g -Wall")

find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  visualization_msgs
  cv_bridge
  message_filters
  message_generation
  common_msgs
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES plan_env
 CATKIN_DEPENDS roscpp std_msgs geometry_msgs common_msgs
 DEPENDS message_generation
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_library(plan_env 
    src/sdf_map.cpp 
    src/raycast.cpp
)
target_link_libraries(plan_env
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
)

# updateESDF3d() parallelizes its distance-transform scanlines with OpenMP;
# without OpenMP the pragmas are ignored and the code runs serially.
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
  target_link_libraries(plan_env OpenMP::OpenMP_CXX)
endif()

if(CATKIN_ENABLE_TESTING)
    catkin_add_gtest(manual_map_layer_test test/manual_map_layer_test.cpp)
    if(TARGET manual_map_layer_test)
        target_link_libraries(manual_map_layer_test
            plan_env
            ${catkin_LIBRARIES}
            ${PCL_LIBRARIES}
            ${OpenCV_LIBS}
        )
    endif()
endif()
