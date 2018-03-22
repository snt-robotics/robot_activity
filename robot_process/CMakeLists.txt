cmake_minimum_required(VERSION 2.8.3)
project(robot_process)

add_compile_options(-std=c++11)
add_compile_options(-Wall)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_srvs
  robot_process_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES robot_process
  CATKIN_DEPENDS roscpp std_srvs robot_process_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(robot_process 
  src/robot_process.cpp 
  src/managed_robot_process.cpp 
  src/subscription_manager.cpp)
add_dependencies(robot_process ${catkin_EXPORTED_TARGETS})
target_link_libraries(robot_process ${catkin_LIBRARIES})
