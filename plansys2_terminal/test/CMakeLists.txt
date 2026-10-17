ament_add_ros_isolated_gtest(terminal_test terminal_test.cpp TIMEOUT 150)
target_link_libraries(terminal_test
  terminal
  readline
  ament_index_cpp::ament_index_cpp
)

ament_add_ros_isolated_gtest(terminal_commands_test terminal_commands_test.cpp TIMEOUT 60)
target_link_libraries(terminal_commands_test
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
