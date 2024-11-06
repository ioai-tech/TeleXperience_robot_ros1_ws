cmake_minimum_required(VERSION 3.0.2)
project(io_teleop_robot_control_node)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  message_generation
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy message_generation message_runtime
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)