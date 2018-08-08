cmake_minimum_required(VERSION 2.8.12)
project(choreonoid_rosplugin)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  trajectory_msgs
  choreonoid
  )

include_directories(
  ${catkin_INCLUDE_DIRS} 
  ${CHOREONOID_INCLUDE_DIRS}
  )

link_directories(
  ${CHOREONOID_LIBRARY_DIRS}
  )

add_subdirectory(src)
