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
    ENV GTEST_FILTER=PlanSetupErrorsTest.${scenario} PLANSYS2_TEST_BT_BUILDER=SimpleBTBuilder
    TIMEOUT 120)
  # The same scenario with STNBTBuilder
  if(NOT scenario STREQUAL "malformed_action_bt_xml")  # A template STNBTBuilder does not use
    ament_add_gtest_test(plan_setup_errors_test
      TEST_NAME plan_setup_errors_${scenario}_stn
      ENV GTEST_FILTER=PlanSetupErrorsTest.${scenario} PLANSYS2_TEST_BT_BUILDER=STNBTBuilder
      TIMEOUT 120)
  endif()
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
    ENV GTEST_FILTER=ExecutorLifecycleTest.${scenario} PLANSYS2_TEST_BT_BUILDER=SimpleBTBuilder
    TIMEOUT 120)
  # The same scenario with STNBTBuilder
  ament_add_gtest_test(executor_lifecycle_test
    TEST_NAME executor_lifecycle_${scenario}_stn
    ENV GTEST_FILTER=ExecutorLifecycleTest.${scenario} PLANSYS2_TEST_BT_BUILDER=STNBTBuilder
    TIMEOUT 120)
endforeach()

# Ctrl-C at different moments of the execution cycle, one process each
foreach(delay 300 410 520 635 750 865 980 1095)
  foreach(builder SimpleBTBuilder STNBTBuilder)
    ament_add_gtest_test(executor_lifecycle_test
      TEST_NAME executor_lifecycle_shutdown_during_a_plan_${delay}ms_${builder}
      ENV GTEST_FILTER=ExecutorLifecycleTest.shutdown_during_a_plan
      PLANSYS2_TEST_BT_BUILDER=${builder} PLANSYS2_TEST_SHUTDOWN_DELAY_MS=${delay}
      TIMEOUT 60)
  endforeach()
endforeach()

ament_add_gtest_executable(executor_goal_handling_test executor_goal_handling_test.cpp)
target_compile_definitions(executor_goal_handling_test PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
target_link_libraries(executor_goal_handling_test ${PROJECT_NAME})
foreach(scenario
    succeeded_plan_reports_only_its_actions failed_plan_is_aborted_with_failure
    cancelled_plan_stops_its_actions cancelled_plan_does_not_start_actions_later
    new_goal_preempts_the_running_one goals_in_a_row_all_end
    cancel_a_goal_waiting_to_start cancel_as_the_plan_ends_does_not_reach_the_next_goal
    failed_action_takes_back_its_at_start_effects
    cancelled_action_takes_back_its_at_start_effects
    replaced_plan_takes_back_its_at_start_effects at_start_effects_stay_if_restore_is_disabled)
  ament_add_gtest_test(executor_goal_handling_test
    TEST_NAME executor_goal_handling_${scenario}
    ENV GTEST_FILTER=ExecutorGoalHandlingTest.${scenario} PLANSYS2_TEST_BT_BUILDER=SimpleBTBuilder
    TIMEOUT 120)
  # The same scenario with STNBTBuilder
  ament_add_gtest_test(executor_goal_handling_test
    TEST_NAME executor_goal_handling_${scenario}_stn
    ENV GTEST_FILTER=ExecutorGoalHandlingTest.${scenario} PLANSYS2_TEST_BT_BUILDER=STNBTBuilder
    TIMEOUT 120)
endforeach()

ament_add_gtest_executable(executor_gone_test executor_gone_test.cpp)
target_compile_definitions(executor_gone_test PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
target_link_libraries(executor_gone_test ${PROJECT_NAME})
foreach(scenario
    PerformerSupervisionTest.performer_stops_when_its_executor_is_gone
    PerformerSupervisionTest.performer_keeps_running_while_its_executor_lives
    ExecutorClientSupervision.client_stops_waiting_when_the_executor_is_gone)
  string(REGEX REPLACE ".*\\." "" short_name ${scenario})
  ament_add_gtest_test(executor_gone_test
    TEST_NAME executor_gone_${short_name}
    ENV GTEST_FILTER=${scenario}
    TIMEOUT 120)
endforeach()

ament_add_gtest(stn_dependency_test stn_dependency_test.cpp)
target_compile_definitions(stn_dependency_test PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
target_link_libraries(stn_dependency_test ${PROJECT_NAME})

ament_add_gtest_executable(executor_stn_test executor_stn_test.cpp)
target_compile_definitions(executor_stn_test PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
target_link_libraries(executor_stn_test ${PROJECT_NAME})
foreach(scenario
    stn_executes_an_action_during_another simple_builder_cannot_run_required_concurrency)
  ament_add_gtest_test(executor_stn_test
    TEST_NAME executor_stn_${scenario}
    ENV GTEST_FILTER=ExecutorStnTest.${scenario}
    TIMEOUT 120)
endforeach()

ament_add_gtest(stn_btbuilder_tests stn_btbuilder_tests.cpp)
target_compile_definitions(stn_btbuilder_tests PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
target_link_libraries(stn_btbuilder_tests
  ${PROJECT_NAME}
  bt_builder_plugins
)

# Tests discover only nodes on this host: parallel jobs do not mix
get_property(_tests DIRECTORY PROPERTY TESTS)
if(_tests)
  set_tests_properties(${_tests}
    PROPERTIES ENVIRONMENT "ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST")
endif()
