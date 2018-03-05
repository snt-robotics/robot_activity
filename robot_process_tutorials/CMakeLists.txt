cmake_minimum_required(VERSION 2.8.3)
project(robot_process_tutorials)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  robot_process
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES robot_process_tutorials
  CATKIN_DEPENDS roscpp robot_process
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(robot_process_tutorials src/robot_process_tutorials.cpp)
target_link_libraries(robot_process_tutorials ${catkin_LIBRARIES})

add_executable(robot_process_tutorials_node src/robot_process_tutorials_node.cpp)
target_link_libraries(robot_process_tutorials_node robot_process_tutorials)
