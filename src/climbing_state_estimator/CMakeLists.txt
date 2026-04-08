cmake_minimum_required(VERSION 3.0.2)
project(climbing_state_estimator)

find_package(catkin REQUIRED COMPONENTS
  climbing_msgs
  geometry_msgs
  rospy
  sensor_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS climbing_msgs geometry_msgs rospy sensor_msgs std_msgs
)

catkin_install_python(PROGRAMS
  scripts/mocap_bridge.py
  scripts/state_estimator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)