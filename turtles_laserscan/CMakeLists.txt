cmake_minimum_required(VERSION 3.5.0)

project(turtles_laserscan CXX)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  message_generation
  roslaunch
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#add_service_files(
#  DIRECTORY srv
#  FILES Kp.srv KpKd.srv
#)
#
#generate_messages(
#  DEPENDENCIES std_msgs
#)

roslaunch_add_file_check(launch)

catkin_package(
  CATKIN_DEPENDS message_runtime
)


