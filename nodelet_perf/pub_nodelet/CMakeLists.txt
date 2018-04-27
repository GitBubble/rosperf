cmake_minimum_required(VERSION 2.8.3)
project(pub_nodelet)

find_package(catkin REQUIRED COMPONENTS nodelet roscpp std_msgs)

## Setup include directories
include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
  LIBRARIES pub_nodelet
  CATKIN_DEPENDS nodelet roscpp std_msgs
)

## Create the nodelet tutorial library
add_library(pub_nodelet src/pub.cpp)
target_link_libraries(pub_nodelet ${catkin_LIBRARIES})
if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(pub_nodelet ${catkin_EXPORTED_LIBRARIES})
endif()

## Mark the nodelet library for installations
install(TARGETS pub_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES pub_nodelet.xml plus.launch plus_default.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
