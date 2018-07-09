cmake_minimum_required(VERSION 2.8.3)
project(rosperf)

if(NOT WIN32)
  set_directory_properties(PROPERTIES COMPILE_OPTIONS "-g;-std=c++11;-Wall;-Wextra")
endif()

find_package(Boost REQUIRED COMPONENTS date_time thread)
find_package(catkin REQUIRED COMPONENTS message_generation rostime roscpp rosconsole roscpp_serialization std_msgs)

include_directories(${catkin_INCLUDE_DIRS} include)
link_directories(${catkin_LIBRARY_DIRS})
add_message_files(DIRECTORY msg FILES perf.msg)
#add_service_files(DIRECTORY srv FILES TwoInts.srv)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)
catkin_package(INCLUDE_DIRS include)

macro(rosperf T)
  add_executable(${T} src/${T}/${T}.cpp)
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

# resource files
install(FILES
  launch/1subscriber_1Publishers_listener.launch
  launch/1subscriber_1Publishers_talker.launch
  launch/1subscriber_1Publishers_ack_talker.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
