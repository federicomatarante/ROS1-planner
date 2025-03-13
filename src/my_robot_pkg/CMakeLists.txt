cmake_minimum_required(VERSION 3.10)

project(my_robot_pkg)

## Find catkin macros and libraries
find_package(OpenCV REQUIRED)
find_package(tf2 REQUIRED)  # Explicitly find the tf2 package
find_package(tf2_geometry_msgs REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  tf2_ros
  nav_msgs
  message_generation
  roslib
  tf2_geometry_msgs  
)

## Specify header file directories
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include  # Explicitly specify include directory path
  ${catkin_INCLUDE_DIRS}
)

## Add custom libraries
file(GLOB SOURCES src/lib/*.cpp)
add_library(my_robot_lib ${SOURCES})

## Generate messages
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  nav_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs nav_msgs tf2_geometry_msgs
)

## Add executables and link libraries
add_executable(planner src/planner.cpp)
target_link_libraries(planner ${catkin_LIBRARIES} my_robot_lib)

add_executable(map_publisher src/map_publisher.cpp)
target_link_libraries(map_publisher ${catkin_LIBRARIES} ${OpenCV_LIBS})
