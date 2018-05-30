cmake_minimum_required(VERSION 3.5.0)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
project(turtles_parking CXX)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  geometry_msgs
  tf
)

catkin_package(
  CATKIN_DEPENDS
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_subdirectory(parking)

add_executable(${PROJECT_NAME}_node src/parking_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  parking
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.hpp"
   PATTERN ".svn" EXCLUDE
 )

