cmake_minimum_required(VERSION 3.0.2)
project(climbing_control_core)

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
  scripts/body_planner.py
  scripts/mission_supervisor.py
  scripts/stance_force_optimizer.py
  scripts/swing_leg_controller.py
  scripts/state_logger.py
  scripts/test_swing_admittance.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)