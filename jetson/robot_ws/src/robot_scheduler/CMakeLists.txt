cmake_minimum_required(VERSION 3.0.2)
project(robot_scheduler)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs           # <---- AQUI você não tinha colocado
  message_generation
)

add_message_files(
  FILES
  SchedulerCommand.msg
  SchedulerResponse.msg
  Object.msg
  Subtask.msg
  TaskRequest.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/scheduler.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
