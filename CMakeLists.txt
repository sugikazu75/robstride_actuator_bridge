cmake_minimum_required(VERSION 3.0.2)
project(robstride_actuator_bridge)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  can_msgs
  message_generation
  roscpp
  sensor_msgs
  std_msgs
  )

## Generate messages in the 'msg' folder
add_message_files(FILES
  MotorCommand.msg
  MotorFeedback.msg
  MotorTorqueCommand.msg
  )

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
  std_msgs
  )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS can_msgs message_runtime roscpp sensor_msgs std_msgs
  )

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

## Declare a cpp library
add_library(robstride_driver
  src/robstride.cpp
  )

target_link_libraries(robstride_driver
  ${catkin_LIBRARIES}
  )

add_library(robstride_controller
  src/motor_config.cpp
)

target_link_libraries(robstride_controller
robstride_driver
${catkin_LIBRARIES}
)

add_executable(robstride_control_test
  src/motor_control_test.cpp
  )

target_link_libraries(robstride_control_test
  robstride_driver
  robstride_controller
  ${catkin_LIBRARIES}
  )
