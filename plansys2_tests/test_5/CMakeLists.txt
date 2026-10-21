install(DIRECTORY pddl
  DESTINATION share/${PROJECT_NAME}/test_5
)

ament_add_ros_isolated_gtest(test_5 test_5.cpp TIMEOUT 80)
target_link_libraries(test_5 ${PROJECT_NAME})

# Tests discover only nodes on this host: parallel jobs do not mix
get_property(_tests DIRECTORY PROPERTY TESTS)
if(_tests)
  set_tests_properties(${_tests}
    PROPERTIES ENVIRONMENT "ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST")
endif()
