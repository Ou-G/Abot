cmake_minimum_required(VERSION 3.0.2)
project(Abot)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
)

catkin_package(
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(straight_cmd src/straight_cmd.cpp)
add_executable(straight_mv src/straight_mv.cpp)
target_link_libraries(straight_cmd ${catkin_LIBRARIES})
target_link_libraries(straight_mv ${catkin_LIBRARIES})