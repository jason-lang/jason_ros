cmake_minimum_required(VERSION 3.0.2)
project(jason_ros)


find_package(catkin REQUIRED COMPONENTS
  jason_ros_msgs
  rospy
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_python_setup()

catkin_install_python(PROGRAMS src/jason_ros/hw_bridge.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
