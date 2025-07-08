cmake_minimum_required(VERSION 3.0.2)
project(robot_scheduler)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

##############################################
## Declare ROS messages and services
##############################################

add_message_files(
  FILES
  SchedulerCommand.msg
  SchedulerResponse.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

##############################################
## catkin specific configuration
##############################################

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)

##############################################
## Build
##############################################

include_directories(
  ${catkin_INCLUDE_DIRS}
)

##############################################
## (Optional) Installation targets
##############################################
# catkin_install_python(PROGRAMS
#   scripts/my_python_script.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
