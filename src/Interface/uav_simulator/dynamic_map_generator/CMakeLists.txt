cmake_minimum_required(VERSION 2.8.3)
project(map_generator)

set(CMAKE_CXX_FLAGS "-std=c++11")
ADD_COMPILE_OPTIONS(-std=c++14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -g -Wall")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  std_srvs
  sensor_msgs
  nav_msgs
  geometry_msgs
  pcl_conversions
  message_generation
  message_runtime
  common_msgs
  visualization_msgs
)
find_package(PCL 1.7 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(octomap REQUIRED)

# 确保消息生成在编译之前
add_message_files(
  FILES
  dynamic_obs.msg
)

add_service_files(
  FILES
  SaveScene.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  common_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    std_srvs
    sensor_msgs
    nav_msgs
    geometry_msgs
    pcl_conversions
    message_runtime
    common_msgs
    visualization_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)
add_executable (local_sensing src/local_sensing.cpp )
  target_link_libraries(local_sensing
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES})

add_executable (map_pub src/map_publisher.cpp )
    target_link_libraries(map_pub
      ${catkin_LIBRARIES}
      ${PCL_LIBRARIES})

add_executable(scene_editor src/scene_editor.cpp)
  add_dependencies(scene_editor
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS})
  target_link_libraries(scene_editor
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES})
