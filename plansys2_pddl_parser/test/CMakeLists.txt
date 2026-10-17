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

# Shared with the tests of the Python parser
install(DIRECTORY
  vectors
  DESTINATION share/${PROJECT_NAME}/test
)

ament_add_gtest(validate_tree_test validate_tree_test.cpp)
target_link_libraries(validate_tree_test ${PROJECT_NAME})

ament_add_gtest(string_utils_test string_utils_test.cpp)
target_link_libraries(string_utils_test ${PROJECT_NAME})

ament_add_gtest(grammar_test grammar_test.cpp)
target_link_libraries(grammar_test ament_index_cpp::ament_index_cpp ${PROJECT_NAME})

ament_add_gtest(vectors_test vectors_test.cpp)
target_link_libraries(vectors_test ${PROJECT_NAME})
target_compile_definitions(vectors_test PRIVATE VECTORS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/vectors")

# Tests discover only nodes on this host: parallel jobs do not mix
get_property(_tests DIRECTORY PROPERTY TESTS)
if(_tests)
  set_tests_properties(${_tests}
    PROPERTIES ENVIRONMENT "ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST")
endif()
