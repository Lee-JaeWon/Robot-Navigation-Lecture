cmake_minimum_required(VERSION 3.0.2)
project(pkg)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  ackermann_msgs
  message_generation
)

add_message_files(
  FILES
  Observation.msg
)

catkin_python_setup()

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
)