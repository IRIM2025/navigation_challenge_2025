cmake_minimum_required(VERSION 3.0.2)
project(maze_generator)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  gazebo_msgs
  std_srvs
  tf
)

## Declare ROS messages, services and actions
## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
#   geometry_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES maze_generator
 CATKIN_DEPENDS rospy std_msgs geometry_msgs gazebo_msgs std_srvs tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
# include
 ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

# Install Python executables
catkin_install_python(PROGRAMS
  scripts/maze_generator.py
  scripts/maze_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

## Install config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  FILES_MATCHING PATTERN "*.yaml"
)

#############
## Testing ##
#############

## Add gtest based cpp tests
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  # add_rostest_gtest(${PROJECT_NAME}-test test/test_maze_generator.test test/test_maze_generator.cpp)
  # target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
endif()
