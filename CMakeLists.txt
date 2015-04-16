cmake_minimum_required(VERSION 2.8.3)
project(carl_safety)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib
  geometry_msgs
  message_generation
  move_base_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  std_srvs
  tf
  wpi_jaco_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(FILES
  Error.msg
)

generate_messages(DEPENDENCIES
)

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(nav_safety src/nav_safety.cpp)
add_executable(teleop_safety src/teleop_safety.cpp)
add_executable(arm_safety src/arm_safety.cpp)
add_executable(tipping_safety src/tipping_safety.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(nav_safety ${catkin_LIBRARIES})
target_link_libraries(teleop_safety ${catkin_LIBRARIES})
target_link_libraries(arm_safety ${catkin_LIBRARIES})
target_link_libraries(tipping_safety ${catkin_LIBRARIES})

add_dependencies(nav_safety ${PROJECT_NAME}_generate_messages_cpp wpi_jaco_msgs_gencpp)
add_dependencies(tipping_safety ${PROJECT_NAME}_generate_messages_cpp wpi_jaco_msgs_gencpp)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS nav_safety teleop_safety arm_safety tipping_safety
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Copy header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)

## Copy launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
