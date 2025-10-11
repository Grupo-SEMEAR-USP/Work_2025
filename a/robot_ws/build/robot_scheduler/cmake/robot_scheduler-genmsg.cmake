# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "robot_scheduler: 5 messages, 0 services")

set(MSG_I_FLAGS "-Irobot_scheduler:/home/rmajetson/Work_2025/a/robot_ws/src/robot_scheduler/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(robot_scheduler_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/rmajetson/Work_2025/a/robot_ws/src/robot_scheduler/msg/SchedulerCommand.msg" NAME_WE)
add_custom_target(_robot_scheduler_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robot_scheduler" "/home/rmajetson/Work_2025/a/robot_ws/src/robot_scheduler/msg/SchedulerCommand.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/rmajetson/Work_2025/a/robot_ws/src/robot_scheduler/msg/SchedulerResponse.msg" NAME_WE)
add_custom_target(_robot_scheduler_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robot_scheduler" "/home/rmajetson/Work_2025/a/robot_ws/src/robot_scheduler/msg/SchedulerResponse.msg" ""
)

get_filename_component(_filename "/home/rmajetson/Work_2025/a/robot_ws/src/robot_scheduler/msg/Object.msg" NAME_WE)
add_custom_target(_robot_scheduler_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robot_scheduler" "/home/rmajetson/Work_2025/a/robot_ws/src/robot_scheduler/msg/Object.msg" "geometry_msgs/Pose:std_msgs/Header:geometry_msgs/Point:geometry_msgs/Quaternion:geometry_msgs/PoseStamped"
)

get_filename_component(_filename "/home/rmajetson/Work_2025/a/robot_ws/src/robot_scheduler/msg/Subtask.msg" NAME_WE)
add_custom_target(_robot_scheduler_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robot_scheduler" "/home/rmajetson/Work_2025/a/robot_ws/src/robot_scheduler/msg/Subtask.msg" "geometry_msgs/Pose:std_msgs/Header:geometry_msgs/Point:geometry_msgs/Quaternion:robot_scheduler/Object:geometry_msgs/PoseStamped"
)

get_filename_component(_filename "/home/rmajetson/Work_2025/a/robot_ws/src/robot_scheduler/msg/TaskRequest.msg" NAME_WE)
add_custom_target(_robot_scheduler_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robot_scheduler" "/home/rmajetson/Work_2025/a/robot_ws/src/robot_scheduler/msg/TaskRequest.msg" "geometry_msgs/Pose:std_msgs/Header:geometry_msgs/Point:geometry_msgs/Quaternion:robot_scheduler/Object:robot_scheduler/Subtask:geometry_msgs/PoseStamped"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(robot_scheduler
  "/home/rmajetson/Work_2025/a/robot_ws/src/robot_scheduler/msg/SchedulerCommand.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robot_scheduler
)
_generate_msg_cpp(robot_scheduler
  "/home/rmajetson/Work_2025/a/robot_ws/src/robot_scheduler/msg/SchedulerResponse.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robot_scheduler
)
_generate_msg_cpp(robot_scheduler
  "/home/rmajetson/Work_2025/a/robot_ws/src/robot_scheduler/msg/Object.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robot_scheduler
)
_generate_msg_cpp(robot_scheduler
  "/home/rmajetson/Work_2025/a/robot_ws/src/robot_scheduler/msg/Subtask.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/rmajetson/Work_2025/a/robot_ws/src/robot_scheduler/msg/Object.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robot_scheduler
)
_generate_msg_cpp(robot_scheduler
  "/home/rmajetson/Work_2025/a/robot_ws/src/robot_scheduler/msg/TaskRequest.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/rmajetson/Work_2025/a/robot_ws/src/robot_scheduler/msg/Object.msg;/home/rmajetson/Work_2025/a/robot_ws/src/robot_scheduler/msg/Subtask.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robot_scheduler
)

### Generating Services

### Generating Module File
_generate_module_cpp(robot_scheduler
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robot_scheduler
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(robot_scheduler_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(robot_scheduler_generate_messages robot_scheduler_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rmajetson/Work_2025/a/robot_ws/src/robot_scheduler/msg/SchedulerCommand.msg" NAME_WE)
add_dependencies(robot_scheduler_generate_messages_cpp _robot_scheduler_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rmajetson/Work_2025/a/robot_ws/src/robot_scheduler/msg/SchedulerResponse.msg" NAME_WE)
add_dependencies(robot_scheduler_generate_messages_cpp _robot_scheduler_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rmajetson/Work_2025/a/robot_ws/src/robot_scheduler/msg/Object.msg" NAME_WE)
add_dependencies(robot_scheduler_generate_messages_cpp _robot_scheduler_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rmajetson/Work_2025/a/robot_ws/src/robot_scheduler/msg/Subtask.msg" NAME_WE)
add_dependencies(robot_scheduler_generate_messages_cpp _robot_scheduler_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rmajetson/Work_2025/a/robot_ws/src/robot_scheduler/msg/TaskRequest.msg" NAME_WE)
add_dependencies(robot_scheduler_generate_messages_cpp _robot_scheduler_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robot_scheduler_gencpp)
add_dependencies(robot_scheduler_gencpp robot_scheduler_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robot_scheduler_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(robot_scheduler
  "/home/rmajetson/Work_2025/a/robot_ws/src/robot_scheduler/msg/SchedulerCommand.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robot_scheduler
)
_generate_msg_eus(robot_scheduler
  "/home/rmajetson/Work_2025/a/robot_ws/src/robot_scheduler/msg/SchedulerResponse.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robot_scheduler
)
_generate_msg_eus(robot_scheduler
  "/home/rmajetson/Work_2025/a/robot_ws/src/robot_scheduler/msg/Object.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robot_scheduler
)
_generate_msg_eus(robot_scheduler
  "/home/rmajetson/Work_2025/a/robot_ws/src/robot_scheduler/msg/Subtask.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/rmajetson/Work_2025/a/robot_ws/src/robot_scheduler/msg/Object.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robot_scheduler
)
_generate_msg_eus(robot_scheduler
  "/home/rmajetson/Work_2025/a/robot_ws/src/robot_scheduler/msg/TaskRequest.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/rmajetson/Work_2025/a/robot_ws/src/robot_scheduler/msg/Object.msg;/home/rmajetson/Work_2025/a/robot_ws/src/robot_scheduler/msg/Subtask.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robot_scheduler
)

### Generating Services

### Generating Module File
_generate_module_eus(robot_scheduler
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robot_scheduler
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(robot_scheduler_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(robot_scheduler_generate_messages robot_scheduler_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rmajetson/Work_2025/a/robot_ws/src/robot_scheduler/msg/SchedulerCommand.msg" NAME_WE)
add_dependencies(robot_scheduler_generate_messages_eus _robot_scheduler_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rmajetson/Work_2025/a/robot_ws/src/robot_scheduler/msg/SchedulerResponse.msg" NAME_WE)
add_dependencies(robot_scheduler_generate_messages_eus _robot_scheduler_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rmajetson/Work_2025/a/robot_ws/src/robot_scheduler/msg/Object.msg" NAME_WE)
add_dependencies(robot_scheduler_generate_messages_eus _robot_scheduler_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rmajetson/Work_2025/a/robot_ws/src/robot_scheduler/msg/Subtask.msg" NAME_WE)
add_dependencies(robot_scheduler_generate_messages_eus _robot_scheduler_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rmajetson/Work_2025/a/robot_ws/src/robot_scheduler/msg/TaskRequest.msg" NAME_WE)
add_dependencies(robot_scheduler_generate_messages_eus _robot_scheduler_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robot_scheduler_geneus)
add_dependencies(robot_scheduler_geneus robot_scheduler_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robot_scheduler_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(robot_scheduler
  "/home/rmajetson/Work_2025/a/robot_ws/src/robot_scheduler/msg/SchedulerCommand.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robot_scheduler
)
_generate_msg_lisp(robot_scheduler
  "/home/rmajetson/Work_2025/a/robot_ws/src/robot_scheduler/msg/SchedulerResponse.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robot_scheduler
)
_generate_msg_lisp(robot_scheduler
  "/home/rmajetson/Work_2025/a/robot_ws/src/robot_scheduler/msg/Object.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robot_scheduler
)
_generate_msg_lisp(robot_scheduler
  "/home/rmajetson/Work_2025/a/robot_ws/src/robot_scheduler/msg/Subtask.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/rmajetson/Work_2025/a/robot_ws/src/robot_scheduler/msg/Object.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robot_scheduler
)
_generate_msg_lisp(robot_scheduler
  "/home/rmajetson/Work_2025/a/robot_ws/src/robot_scheduler/msg/TaskRequest.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/rmajetson/Work_2025/a/robot_ws/src/robot_scheduler/msg/Object.msg;/home/rmajetson/Work_2025/a/robot_ws/src/robot_scheduler/msg/Subtask.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robot_scheduler
)

### Generating Services

### Generating Module File
_generate_module_lisp(robot_scheduler
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robot_scheduler
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(robot_scheduler_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(robot_scheduler_generate_messages robot_scheduler_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rmajetson/Work_2025/a/robot_ws/src/robot_scheduler/msg/SchedulerCommand.msg" NAME_WE)
add_dependencies(robot_scheduler_generate_messages_lisp _robot_scheduler_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rmajetson/Work_2025/a/robot_ws/src/robot_scheduler/msg/SchedulerResponse.msg" NAME_WE)
add_dependencies(robot_scheduler_generate_messages_lisp _robot_scheduler_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rmajetson/Work_2025/a/robot_ws/src/robot_scheduler/msg/Object.msg" NAME_WE)
add_dependencies(robot_scheduler_generate_messages_lisp _robot_scheduler_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rmajetson/Work_2025/a/robot_ws/src/robot_scheduler/msg/Subtask.msg" NAME_WE)
add_dependencies(robot_scheduler_generate_messages_lisp _robot_scheduler_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rmajetson/Work_2025/a/robot_ws/src/robot_scheduler/msg/TaskRequest.msg" NAME_WE)
add_dependencies(robot_scheduler_generate_messages_lisp _robot_scheduler_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robot_scheduler_genlisp)
add_dependencies(robot_scheduler_genlisp robot_scheduler_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robot_scheduler_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(robot_scheduler
  "/home/rmajetson/Work_2025/a/robot_ws/src/robot_scheduler/msg/SchedulerCommand.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robot_scheduler
)
_generate_msg_nodejs(robot_scheduler
  "/home/rmajetson/Work_2025/a/robot_ws/src/robot_scheduler/msg/SchedulerResponse.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robot_scheduler
)
_generate_msg_nodejs(robot_scheduler
  "/home/rmajetson/Work_2025/a/robot_ws/src/robot_scheduler/msg/Object.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robot_scheduler
)
_generate_msg_nodejs(robot_scheduler
  "/home/rmajetson/Work_2025/a/robot_ws/src/robot_scheduler/msg/Subtask.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/rmajetson/Work_2025/a/robot_ws/src/robot_scheduler/msg/Object.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robot_scheduler
)
_generate_msg_nodejs(robot_scheduler
  "/home/rmajetson/Work_2025/a/robot_ws/src/robot_scheduler/msg/TaskRequest.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/rmajetson/Work_2025/a/robot_ws/src/robot_scheduler/msg/Object.msg;/home/rmajetson/Work_2025/a/robot_ws/src/robot_scheduler/msg/Subtask.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robot_scheduler
)

### Generating Services

### Generating Module File
_generate_module_nodejs(robot_scheduler
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robot_scheduler
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(robot_scheduler_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(robot_scheduler_generate_messages robot_scheduler_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rmajetson/Work_2025/a/robot_ws/src/robot_scheduler/msg/SchedulerCommand.msg" NAME_WE)
add_dependencies(robot_scheduler_generate_messages_nodejs _robot_scheduler_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rmajetson/Work_2025/a/robot_ws/src/robot_scheduler/msg/SchedulerResponse.msg" NAME_WE)
add_dependencies(robot_scheduler_generate_messages_nodejs _robot_scheduler_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rmajetson/Work_2025/a/robot_ws/src/robot_scheduler/msg/Object.msg" NAME_WE)
add_dependencies(robot_scheduler_generate_messages_nodejs _robot_scheduler_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rmajetson/Work_2025/a/robot_ws/src/robot_scheduler/msg/Subtask.msg" NAME_WE)
add_dependencies(robot_scheduler_generate_messages_nodejs _robot_scheduler_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rmajetson/Work_2025/a/robot_ws/src/robot_scheduler/msg/TaskRequest.msg" NAME_WE)
add_dependencies(robot_scheduler_generate_messages_nodejs _robot_scheduler_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robot_scheduler_gennodejs)
add_dependencies(robot_scheduler_gennodejs robot_scheduler_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robot_scheduler_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(robot_scheduler
  "/home/rmajetson/Work_2025/a/robot_ws/src/robot_scheduler/msg/SchedulerCommand.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_scheduler
)
_generate_msg_py(robot_scheduler
  "/home/rmajetson/Work_2025/a/robot_ws/src/robot_scheduler/msg/SchedulerResponse.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_scheduler
)
_generate_msg_py(robot_scheduler
  "/home/rmajetson/Work_2025/a/robot_ws/src/robot_scheduler/msg/Object.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_scheduler
)
_generate_msg_py(robot_scheduler
  "/home/rmajetson/Work_2025/a/robot_ws/src/robot_scheduler/msg/Subtask.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/rmajetson/Work_2025/a/robot_ws/src/robot_scheduler/msg/Object.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_scheduler
)
_generate_msg_py(robot_scheduler
  "/home/rmajetson/Work_2025/a/robot_ws/src/robot_scheduler/msg/TaskRequest.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/rmajetson/Work_2025/a/robot_ws/src/robot_scheduler/msg/Object.msg;/home/rmajetson/Work_2025/a/robot_ws/src/robot_scheduler/msg/Subtask.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_scheduler
)

### Generating Services

### Generating Module File
_generate_module_py(robot_scheduler
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_scheduler
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(robot_scheduler_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(robot_scheduler_generate_messages robot_scheduler_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rmajetson/Work_2025/a/robot_ws/src/robot_scheduler/msg/SchedulerCommand.msg" NAME_WE)
add_dependencies(robot_scheduler_generate_messages_py _robot_scheduler_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rmajetson/Work_2025/a/robot_ws/src/robot_scheduler/msg/SchedulerResponse.msg" NAME_WE)
add_dependencies(robot_scheduler_generate_messages_py _robot_scheduler_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rmajetson/Work_2025/a/robot_ws/src/robot_scheduler/msg/Object.msg" NAME_WE)
add_dependencies(robot_scheduler_generate_messages_py _robot_scheduler_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rmajetson/Work_2025/a/robot_ws/src/robot_scheduler/msg/Subtask.msg" NAME_WE)
add_dependencies(robot_scheduler_generate_messages_py _robot_scheduler_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rmajetson/Work_2025/a/robot_ws/src/robot_scheduler/msg/TaskRequest.msg" NAME_WE)
add_dependencies(robot_scheduler_generate_messages_py _robot_scheduler_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robot_scheduler_genpy)
add_dependencies(robot_scheduler_genpy robot_scheduler_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robot_scheduler_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robot_scheduler)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robot_scheduler
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(robot_scheduler_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(robot_scheduler_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robot_scheduler)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robot_scheduler
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(robot_scheduler_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(robot_scheduler_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robot_scheduler)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robot_scheduler
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(robot_scheduler_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(robot_scheduler_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robot_scheduler)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robot_scheduler
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(robot_scheduler_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(robot_scheduler_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_scheduler)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_scheduler\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_scheduler
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(robot_scheduler_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(robot_scheduler_generate_messages_py geometry_msgs_generate_messages_py)
endif()
