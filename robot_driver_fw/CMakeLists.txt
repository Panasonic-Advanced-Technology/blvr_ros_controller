cmake_minimum_required(VERSION 3.0.2)
project(robot_driver_fw)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 14)

add_compile_options(-Wall -Wextra -Werror)

# for switch modbus
if($ENV{ROS_DISTRO} STREQUAL "melodic")
    add_definitions(-DROS_MELODIC)
elseif($ENV{ROS_DISTRO}  STREQUAL "noetic")
    add_definitions(-DROS_NOETIC)
else()
    message(FATAL_ERROR "Unsupported ROS distribution: ${ROS_DISTRO}")
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp topic_tools tf2 tf_conversions nav_msgs geometry_msgs robot_driver_msgs)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR})
find_package(Modbus REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
#add_message_files(
#  FILES
#)

#generate_messages(
#  DEPENDENCIES
#  std_msgs
#)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
  tf_conversions
  nav_msgs
  geometry_msgs
  robot_driver_msgs
  std_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${MODBUS_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
 src/robot_driver_ros.cpp
 src/robot_driver.cpp
 src/odometry.cpp
 src/robot_status.cpp
 src/modbus_wrapper.cpp
 src/mecanum_robot_driver_ros.cpp
 src/mecanum_robot_driver.cpp
 src/mecanum_odometry.cpp
)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${MODBUS_LIBRARIES}
)

#############
## Install ##
#############

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
