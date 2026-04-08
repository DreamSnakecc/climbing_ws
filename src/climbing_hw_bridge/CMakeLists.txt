cmake_minimum_required(VERSION 3.0.2)
project(climbing_hw_bridge)

find_package(catkin REQUIRED COMPONENTS
  climbing_msgs
  dynamixel_control
  geometry_msgs
  rospy
  sensor_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS climbing_msgs dynamixel_control geometry_msgs rospy sensor_msgs std_msgs
)

catkin_install_python(PROGRAMS
  scripts/dynamixel_bridge.py
  scripts/fan_serial_bridge.py
  scripts/imu_serial_bridge.py
  scripts/leg_ik_executor.py
  scripts/local_safety_supervisor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)