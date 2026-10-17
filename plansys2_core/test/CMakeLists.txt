ament_add_gtest(utils_test utils_test.cpp)
target_link_libraries(utils_test
  ${PROJECT_NAME}
  ament_index_cpp::ament_index_cpp
)

ament_add_gtest(plan_solver_base_test plan_solver_base_test.cpp TIMEOUT 120)
target_link_libraries(plan_solver_base_test ${PROJECT_NAME})

ament_add_ros_isolated_gtest(test_utils_test test_utils_test.cpp TIMEOUT 60)
target_link_libraries(test_utils_test ${PROJECT_NAME} ${std_msgs_TARGETS})

ament_add_gtest(pddl_requirements_test pddl_requirements_test.cpp)
target_link_libraries(pddl_requirements_test ${PROJECT_NAME})

ament_add_gtest(types_test types_test.cpp)
target_link_libraries(types_test ${PROJECT_NAME})

ament_add_gtest(node_variant_test node_variant_test.cpp)
target_link_libraries(node_variant_test ${PROJECT_NAME})

ament_add_gtest(graph_test graph_test.cpp)
target_link_libraries(graph_test ${PROJECT_NAME})

ament_add_gtest(state_test state_test.cpp)
target_link_libraries(state_test ${PROJECT_NAME})

ament_add_gtest(state_consistency_test state_consistency_test.cpp)
target_link_libraries(state_consistency_test ${PROJECT_NAME})

set(TEST_PDDL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/pddl)

install(DIRECTORY
  ${TEST_PDDL_DIR}
  DESTINATION share/${PROJECT_NAME}
)

# Tests discover only nodes on this host: parallel jobs do not mix
get_property(_tests DIRECTORY PROPERTY TESTS)
if(_tests)
  set_tests_properties(${_tests}
    PROPERTIES ENVIRONMENT "ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST")
endif()
