cmake_minimum_required(VERSION 2.8.3)
project(mock_loop)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  geometry_msgs
  eigen_conversions
)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package()

include_directories( 
  SYSTEM 
  # include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
)

add_executable(mock_loop_node
    src/mock_loop.cpp 
)

target_link_libraries(mock_loop_node 
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)