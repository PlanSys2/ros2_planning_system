ament_add_gtest(planner_test planner_test.cpp)
target_link_libraries(planner_test
  ${PROJECT_NAME}
  ament_index_cpp::ament_index_cpp
  dl
  plansys2_pddl_parser::plansys2_pddl_parser
)
target_compile_definitions(planner_test PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
ament_add_gtest(planner_node_errors_test planner_node_errors_test.cpp TIMEOUT 120)
target_link_libraries(planner_node_errors_test
  ${PROJECT_NAME}
  ament_index_cpp::ament_index_cpp
)
target_compile_definitions(planner_node_errors_test PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
