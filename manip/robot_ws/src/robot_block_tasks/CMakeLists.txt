cmake_minimum_required(VERSION 3.0.2)
project(robot_block_tasks)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  apriltag_ros
)

catkin_package(
  CATKIN_DEPENDS std_msgs rospy geometry_msgs apriltag_ros
)

catkin_install_python(PROGRAMS
  scripts/align_block.py
  scripts/search_line.py
  scripts/table_orientation.py
  scripts/mapping_ids.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
