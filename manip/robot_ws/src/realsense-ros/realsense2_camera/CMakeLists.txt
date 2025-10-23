cmake_minimum_required(VERSION 2.8.3)
project(realsense2_camera)

option(BUILD_WITH_OPENMP "Use OpenMP" OFF)
option(SET_USER_BREAK_AT_STARTUP "Set user wait point in startup (for debug)" OFF)

add_definitions(-D_CRT_SECURE_NO_WARNINGS)
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
  nodelet
  cv_bridge
  image_transport
  tf
  ddynamic_reconfigure
  diagnostic_updater
)

# 3. Biblioteca externa OpenCV  ➜ chamada SEPARADA
find_package(OpenCV 4 REQUIRED)          # ou "find_package(OpenCV REQUIRED)"

# (mantém seus blocos de OpenMP / debug / flags etc.)

# ------------------------------------------------------------------
# 4. RealSense SDK
if (WIN32)
  find_package(realsense2 CONFIG REQUIRED)
else()
  find_package(realsense2 2.50.0 REQUIRED)
endif()

# ------------------------------------------------------------------
# 5. Mensagens e serviços
add_message_files(FILES
  IMUInfo.msg
  Extrinsics.msg
  Metadata.msg
)
add_service_files(FILES DeviceInfo.srv)
generate_messages(DEPENDENCIES sensor_msgs std_msgs)

# ------------------------------------------------------------------
# 6. Include paths
include_directories(
  include
  ${realsense2_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# ------------------------------------------------------------------
# 7. Biblioteca / nodelet
add_library(${PROJECT_NAME}
  include/realsense2_camera/constants.h
  include/realsense2_camera/realsense_node_factory.h
  include/realsense2_camera/base_realsense_node.h
  include/realsense2_camera/t265_realsense_node.h
  src/realsense_node_factory.cpp
  src/base_realsense_node.cpp
  src/t265_realsense_node.cpp
)
add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  ${realsense2_LIBRARY}        # ou realsense2::realsense2 se preferir
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${CMAKE_THREAD_LIBS_INIT}
)

# ------------------------------------------------------------------
# 8. Anúncio do pacote para o Catkin
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES    ${PROJECT_NAME}
  CATKIN_DEPENDS
      message_runtime
      roscpp sensor_msgs std_msgs
      nodelet cv_bridge image_transport
      ddynamic_reconfigure nav_msgs
)

# ------------------------------------------------------------------
# 9. Instalação (mantém como já estava)
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/realsense2_camera
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
)
install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
