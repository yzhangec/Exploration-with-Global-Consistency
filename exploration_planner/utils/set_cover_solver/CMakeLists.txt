cmake_minimum_required(VERSION 2.8.3)
project(set_cover_solver)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES set_cover_solver
 CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories( 
    SYSTEM 
    include 
)

file( GLOB SRCS src/*.cpp )

# message("${SRCS}")

add_library( set_cover_solver 
  ${SRCS}
  src/set_cover_solver.cpp
  src/set_cover_solver_test.cpp
)
target_link_libraries(set_cover_solver -lm)

add_executable( set_cover
  ${SRCS}
)
target_link_libraries(set_cover ${catkin_LIBRARIES} -lm)