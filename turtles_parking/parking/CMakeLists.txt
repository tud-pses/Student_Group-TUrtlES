add_library(
  parking
  src/CParking.cpp
  src/CSensors.cpp
  src/CMovement.cpp
  src/CParking.cpp
  src/CPIDController.cpp
  src/CScouting.cpp
  src/CParkProcess.cpp
  )

target_include_directories(
  parking 
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/parking>
  )

target_link_libraries(
  parking
  ${catkin_LIBRARIES}
  )
