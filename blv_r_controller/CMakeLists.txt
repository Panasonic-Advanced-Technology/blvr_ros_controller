cmake_minimum_required(VERSION 3.0.2)
project(blv_r_controller)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 14)

add_compile_options(-Wall -Wextra -Werror)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp topic_tools tf2 sensor_msgs tf_conversions nav_msgs geometry_msgs robot_driver_msgs robot_driver_fw)

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
  CATKIN_DEPENDS
  sensor_msgs
  tf_conversions
  nav_msgs
  geometry_msgs
  robot_driver_msgs
  std_msgs
  message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}
 src/blv_r_battery.cpp
 src/blv_r_modbus_id_share.cpp
 src/blv_r_controller.cpp
 src/blv_r_manager.cpp
 src/blv_r_manager_main.cpp
)
add_executable(blv_r_mecanum_controller
  src/blv_r_battery.cpp
  src/blv_r_modbus_id_share.cpp
  src/blv_r_4wd_modbus_id_share.cpp
  src/blv_r_controller.cpp
  src/blv_r_4wd_controller.cpp
  src/blv_r_manager.cpp
  src/blv_r_4wd_manager.cpp
  src/blv_r_mecanum_main.cpp
)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(blv_r_mecanum_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
target_link_libraries(blv_r_mecanum_controller
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
install(TARGETS ${PROJECT_NAME} blv_r_mecanum_controller
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## Install files
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
