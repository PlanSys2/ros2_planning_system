ament_add_gtest(pddl_parser_test pddl_parser_test.cpp TIMEOUT 120)
target_link_libraries(pddl_parser_test
  ament_index_cpp::ament_index_cpp
  rclcpp::rclcpp
  ${PROJECT_NAME}
)

install(DIRECTORY
  pddl
  DESTINATION share/${PROJECT_NAME}
)

ament_add_gtest(validate_tree_test validate_tree_test.cpp)
target_link_libraries(validate_tree_test ${PROJECT_NAME})

ament_add_gtest(string_utils_test string_utils_test.cpp)
target_link_libraries(string_utils_test ${PROJECT_NAME})

# Tests discover only nodes on this host: parallel jobs do not mix
get_property(_tests DIRECTORY PROPERTY TESTS)
if(_tests)
  set_tests_properties(${_tests}
    PROPERTIES ENVIRONMENT "ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST")
endif()
