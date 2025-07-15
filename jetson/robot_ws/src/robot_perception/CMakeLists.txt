cmake_minimum_required(VERSION 3.0.2)
project(robot_perception)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_generation
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robot_perception
#  CATKIN_DEPENDS cv_bridge roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_message_files(
  FILES
  Identify.msg
  Point.msg
)

catkin_install_python(PROGRAMS
  scripts/vision_identification.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)