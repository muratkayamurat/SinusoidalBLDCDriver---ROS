cmake_minimum_required(VERSION 3.0.2)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

project(sinusoidal)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs sensor_msgs rospy geometry_msgs)

generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
  geometry_msgs
)
catkin_package(CATKIN_DEPENDS message_runtime std_msgs sensor_msgs)



catkin_install_python(PROGRAMS nodes/motordriver.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
