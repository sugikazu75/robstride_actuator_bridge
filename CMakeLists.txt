cmake_minimum_required(VERSION 3.0.2)
project(motor_control)

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
  )

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
  std_msgs
  )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS message_runtime roscpp sensor_msgs std_msgs
  )

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

## Declare a cpp library
add_library(${PROJECT_NAME}
  src/motor_config.cpp
  src/robstride.cpp
  )

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  )

add_executable(motor_control_test
  src/motor_control_test.cpp
  )

add_dependencies(motor_control_test
  ${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(motor_control_test
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  )

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  )
