ament_add_gtest(terminal_test terminal_test.cpp TIMEOUT 500)
target_link_libraries(terminal_test
  terminal
  readline
  ament_index_cpp::ament_index_cpp
)

install(DIRECTORY
  pddl
  DESTINATION share/${PROJECT_NAME}
)

# Tests discover only nodes on this host: parallel jobs do not mix
get_property(_tests DIRECTORY PROPERTY TESTS)
if(_tests)
  set_tests_properties(${_tests}
    PROPERTIES ENVIRONMENT "ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST")
endif()
