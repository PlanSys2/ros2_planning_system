ament_add_gtest(utils_test utils_test.cpp)
target_link_libraries(utils_test
  ${PROJECT_NAME}
  ament_index_cpp::ament_index_cpp
)

ament_add_gtest(types_test types_test.cpp)
target_link_libraries(types_test ${PROJECT_NAME})

ament_add_gtest(node_variant_test node_variant_test.cpp)
target_link_libraries(node_variant_test ${PROJECT_NAME})

ament_add_gtest(graph_test graph_test.cpp)
target_link_libraries(graph_test ${PROJECT_NAME})

ament_add_gtest(state_test state_test.cpp)
target_link_libraries(state_test ${PROJECT_NAME})

set(TEST_PDDL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/pddl)

install(DIRECTORY
  ${TEST_PDDL_DIR}
  DESTINATION share/${PROJECT_NAME}
)
