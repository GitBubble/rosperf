cmake_minimum_required(VERSION 2.8.3)
project(rosperf)

if(NOT WIN32)
  set_directory_properties(PROPERTIES COMPILE_OPTIONS "-g;-std=c++11;-Wall;-Wextra")
endif()

find_package(Boost REQUIRED COMPONENTS date_time thread)
find_package(catkin REQUIRED COMPONENTS message_generation nodelet rostime roscpp rosconsole roscpp_serialization std_msgs)

include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})
add_message_files(DIRECTORY msg FILES perf.msg)
#add_service_files(DIRECTORY srv FILES TwoInts.srv)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)

macro(rosperf T)
  add_executable(${T} ${T}/${T}.cpp)
  target_link_libraries(${T} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
  add_dependencies(${T} rosperf_gencpp)
  install(TARGETS ${T}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endmacro()

foreach(dir
    perf_talker
    perf_listener
    )
  rosperf(${dir})
endforeach()

# process nodelet

macro(perf_nodelet T)
  add_library(${T} ${T}/${T}.cpp)
  target_link_libraries(${T} ${catkin_LIBRARIES})
  if(catkin_EXPORTED_LIBRARIES)
    add_dependencies(${T} ${catkin_EXPORTED_LIBRARIES})
  endif()
 install(TARGETS ${T}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endmacro()

foreach(dir
    pub_nodelet
    sub_nodelet
    )
  perf_nodelet(${dir})
endforeach()


# resource files
install(FILES
  launch/20subscriber1000.launch
  launch/20subscriber1000_screen.launch
  launch/20subscriber1000_screen_nodebug.launch
  launch/sub_nodelet.xml
  launch/plus.launch
  launch/plus_default.yaml
  launch/pub_nodelet.xml  
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
