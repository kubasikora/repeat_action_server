cmake_minimum_required(VERSION 3.0.2)
project(repeat_action_server)

find_package(catkin REQUIRED COMPONENTS actionlib roscpp tf)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS actionlib roscpp tf
)

include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(${PROJECT_NAME} src/main.cxx)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
