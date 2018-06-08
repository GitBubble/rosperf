include_directories(${catkin_INCLUDE_DIRS})
add_executable(perf_talker perf_talker.cpp)
target_link_libraries(perf_talker ${catkin_LIBRARIES})
install(TARGETS perf_talker
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
