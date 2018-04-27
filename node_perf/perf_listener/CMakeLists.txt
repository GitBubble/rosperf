include_directories(${catkin_INCLUDE_DIRS})
add_executable(perf_listener perf_listener.cpp)
target_link_libraries(perf_listener ${catkin_LIBRARIES})
install(TARGETS perf_listener
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
