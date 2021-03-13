ament_add_gtest(executor_test executor_test.cpp)
target_compile_definitions(executor_test PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
target_link_libraries(executor_test ${PROJECT_NAME})

ament_add_gtest(action_execution_test action_execution_test.cpp)
target_compile_definitions(action_execution_test PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
target_link_libraries(action_execution_test ${PROJECT_NAME})


ament_add_gtest(execution_tree_test execution_tree_test.cpp)
target_compile_definitions(execution_tree_test PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
target_link_libraries(execution_tree_test ${PROJECT_NAME})
ament_target_dependencies(execution_tree_test ${dependencies})

ament_add_gtest(btbuilder_tests btbuilder_tests.cpp)
target_compile_definitions(btbuilder_tests PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
target_link_libraries(btbuilder_tests ${PROJECT_NAME})
ament_target_dependencies(btbuilder_tests ${dependencies})

ament_add_gtest(bt_node_test bt_node_test.cpp)
target_compile_definitions(bt_node_test PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
target_link_libraries(bt_node_test ${PROJECT_NAME})
ament_target_dependencies(bt_node_test ${dependencies})

ament_add_gtest(bt_node_test_charging bt_node_test_charging.cpp)
target_compile_definitions(bt_node_test_charging PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
target_link_libraries(bt_node_test_charging ${PROJECT_NAME})
ament_target_dependencies(bt_node_test_charging ${dependencies})
