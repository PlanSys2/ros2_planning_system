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
