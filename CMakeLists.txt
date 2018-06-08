cmake_minimum_required(VERSION 2.8.3)
project(rosperf)

if(NOT WIN32)
  set_directory_properties(PROPERTIES COMPILE_OPTIONS "-std=c++11;-D_GLIBCXX_USE_CXX11_ABI=1;-Wall;-Wextra")
endif()

find_package(Boost REQUIRED COMPONENTS date_time thread)
find_package(catkin REQUIRED COMPONENTS message_generation rostime roscpp rosconsole roscpp_serialization)

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


install(FILES
  launch/talker_listener.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
