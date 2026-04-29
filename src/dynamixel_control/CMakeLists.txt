cmake_minimum_required(VERSION 3.0.2)
project(dynamixel_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  dynamixel_sdk
)

## Generate messages and services
add_message_files(
  FILES
  SetCurrent.msg
  SetOperatingMode.msg
  SetPosition.msg
)

add_service_files(
  FILES
  GetPosition.srv
  GetCurrent.srv
  GetBulkPositions.srv
  GetBulkCurrents.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime dynamixel_sdk
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(multi_dxl_node src/multi_dxl_node.cpp)
add_dependencies(multi_dxl_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(multi_dxl_node
  ${catkin_LIBRARIES}
)

