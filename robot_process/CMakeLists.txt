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
  src/resource/managed_resource.cpp
  src/resource/resource_manager.cpp
)
add_dependencies(robot_process ${catkin_EXPORTED_TARGETS})
target_link_libraries(robot_process ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)

  find_package(rostest REQUIRED)

  add_rostest_gtest(
    robot_process_tests
    test/robot_process.test 
    test/robot_process_tests.cpp)

  target_link_libraries(robot_process_tests robot_process ${catkin_LIBRARIES})

endif()