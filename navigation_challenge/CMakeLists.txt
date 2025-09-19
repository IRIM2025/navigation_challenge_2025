cmake_minimum_required(VERSION 3.0.2)
project(navigation_challenge)


find_package(catkin REQUIRED COMPONENTS
  maze_generator
  pedsim_gazebo_plugin
  pedsim_simulator
  joy_teleop
  key_teleop
  mouse_teleop
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

