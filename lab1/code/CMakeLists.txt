cmake_minimum_required(VERSION 2.8.3)
project(sensor_processor)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

catkin_package()
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/sensor_processor.cpp)
add_executable(${PROJECT_NAME}_node src/sensor_processor_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
