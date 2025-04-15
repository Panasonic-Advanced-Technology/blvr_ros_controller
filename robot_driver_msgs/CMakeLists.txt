cmake_minimum_required(VERSION 2.8.3)
project(robot_driver_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  MotionParam.msg
  MotorStatus.msg
  MotorError.msg
  MotorStatusComplex.msg
)

add_service_files(
  FILES
  ConfigSetPos.srv
  GetMotorStatus.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  geometry_msgs
)
