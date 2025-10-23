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

add_message_files(
  FILES
  Identify.msg
  Point.msg
  UltrasonicDistances.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs sensor_msgs rospy roscpp cv_bridge
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/vision_identification.py
  scripts/i2c_ultrasonic.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
