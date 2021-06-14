cmake_minimum_required(VERSION 3.0.2)
project(repeat_action_server)

find_package(catkin REQUIRED COMPONENTS actionlib roscpp tf nav_msgs move_base_msgs)

add_action_files(DIRECTORY actions FILES AskToRepeat.action)
generate_messages(DEPENDENCIES actionlib_msgs std_msgs)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS actionlib roscpp tf nav_msgs move_base_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(${PROJECT_NAME} src/main.cxx src/AskToRepeatActionServer.cxx src/util.cxx)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
