cmake_minimum_required(VERSION 2.8.3)
project(jason_hw_bridge)


find_package(catkin REQUIRED COMPONENTS
  jason_msgs
  rospy
)


catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
