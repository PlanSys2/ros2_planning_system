ament_add_gtest(utils_test utils_test.cpp)
target_link_libraries(utils_test
  ${PROJECT_NAME}
  ament_index_cpp::ament_index_cpp
)

ament_add_gtest(problem_expert_test problem_expert_test.cpp)
target_link_libraries(problem_expert_test
  ${PROJECT_NAME}
  ament_index_cpp::ament_index_cpp
)

ament_add_gtest(problem_expert_node_test problem_expert_node_test.cpp)
target_link_libraries(problem_expert_node_test
  ${PROJECT_NAME}
  ament_index_cpp::ament_index_cpp
)

ament_add_gtest(malformed_pddl_test malformed_pddl_test.cpp TIMEOUT 300)
target_link_libraries(malformed_pddl_test
  ${PROJECT_NAME}
  ament_index_cpp::ament_index_cpp
)

ament_add_gtest(tree_node_ids_test tree_node_ids_test.cpp TIMEOUT 300)
target_link_libraries(tree_node_ids_test
  ${PROJECT_NAME}
  ament_index_cpp::ament_index_cpp
)
