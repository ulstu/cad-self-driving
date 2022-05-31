cmake_minimum_required(VERSION 2.8.3)
project(sdgazelle)

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  roscpp
  rospy
  std_msgs
  genmsg
  image_transport
)

catkin_package(
	#  INCLUDE_DIRS include
  LIBRARIES sdgazelle
  CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

