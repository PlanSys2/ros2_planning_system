ament_add_gtest(executor_test executor_test.cpp TIMEOUT 300)
target_compile_definitions(executor_test
  PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
target_link_libraries(executor_test
  ${PROJECT_NAME}
)

ament_add_gtest(action_execution_test action_execution_test.cpp)
target_compile_definitions(action_execution_test
  PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
target_link_libraries(action_execution_test
  ${PROJECT_NAME}
)

ament_add_gtest(execution_tree_test execution_tree_test.cpp)
target_compile_definitions(execution_tree_test
  PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
target_link_libraries(execution_tree_test
  ${PROJECT_NAME}
)

ament_add_gtest(simple_btbuilder_tests simple_btbuilder_tests.cpp)
target_compile_definitions(simple_btbuilder_tests
  PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
target_link_libraries(simple_btbuilder_tests
  ${PROJECT_NAME}
  bt_builder_plugins
)

ament_add_gtest(sequential_bt_builder_test sequential_bt_builder_test.cpp)
target_compile_definitions(sequential_bt_builder_test
  PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
target_link_libraries(sequential_bt_builder_test
  ${PROJECT_NAME}
  bt_builder_plugins
)

ament_add_gtest(bt_node_test bt_node_test.cpp)
target_compile_definitions(bt_node_test
  PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
target_link_libraries(bt_node_test
  ${PROJECT_NAME}
)

ament_add_gtest(bt_node_test_charging bt_node_test_charging.cpp)
target_compile_definitions(bt_node_test_charging
  PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
target_link_libraries(bt_node_test_charging
  ${PROJECT_NAME}
)

# One process per scenario: a failing setup must not take the others down, and the
# executor node of one scenario must not leak into the next (#430)
ament_add_gtest_executable(plan_setup_errors_test plan_setup_errors_test.cpp)
target_compile_definitions(plan_setup_errors_test PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
target_link_libraries(plan_setup_errors_test ${PROJECT_NAME})
foreach(scenario
    valid_plan empty_plan unknown_action unknown_action_among_valid_ones malformed_action
    unknown_bt_builder_plugin malformed_action_bt_xml sequential_bt_builder)
  ament_add_gtest_test(plan_setup_errors_test
    TEST_NAME plan_setup_errors_${scenario}
    ENV GTEST_FILTER=PlanSetupErrorsTest.${scenario}
    TIMEOUT 120)
endforeach()

ament_add_gtest_executable(executor_lifecycle_test executor_lifecycle_test.cpp)
target_compile_definitions(executor_lifecycle_test PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
target_link_libraries(executor_lifecycle_test ${PROJECT_NAME})
foreach(scenario
    node_is_destroyed_after_a_plan node_is_destroyed_while_active goals_rejected_unless_active
    deactivate_during_a_plan_fails_it reactivations_run_a_single_execution_thread
    cleanup_and_configure_again plan_services_during_execution)
  ament_add_gtest_test(executor_lifecycle_test
    TEST_NAME executor_lifecycle_${scenario}
    ENV GTEST_FILTER=ExecutorLifecycleTest.${scenario}
    TIMEOUT 120)
endforeach()
