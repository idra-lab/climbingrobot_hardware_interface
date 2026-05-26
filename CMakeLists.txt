cmake_minimum_required(VERSION 3.0.2)
project(climbingrobot_hardware_interface)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf
  message_generation
)

################################################
## Messages and services
################################################


add_message_files(
  DIRECTORY
  msg
  FILES
  DebugMessage.msg
  Imus.msg
  RopeCommand.msg
  RopeTelemetry.msg
  PropellerCommand.msg
  AlpineBodyTelemetry.msg
)

add_service_files(
  DIRECTORY
  srv
  FILES
  AlpineBodyCommand.srv
  RopeControlMode.srv
)


generate_messages(
  DEPENDENCIES
  std_msgs geometry_msgs
)





 


###################################
## Catkin package configuration
###################################

catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
    tf
    message_runtime
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

catkin_install_python(PROGRAMS
  scripts/alpine_odometry_node.py
  scripts/dongle_node.py
  scripts/friction_estimator.py
  scripts/homing_procedure.py
  scripts/jump.py
  scripts/position_control_logger.py
  scripts/position_step_test.py
  scripts/telemetry_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  launch
  scripts
  config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
