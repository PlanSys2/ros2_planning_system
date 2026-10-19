ament_add_ros_isolated_gtest(planner_test planner_test.cpp TIMEOUT 60)
target_link_libraries(planner_test
  ${PROJECT_NAME}
  ament_index_cpp::ament_index_cpp
  dl
  plansys2_pddl_parser::plansys2_pddl_parser
)
target_compile_definitions(planner_test PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
ament_add_ros_isolated_gtest(planner_node_errors_test planner_node_errors_test.cpp TIMEOUT 60)
target_link_libraries(planner_node_errors_test
  ${PROJECT_NAME}
  ament_index_cpp::ament_index_cpp
)
target_compile_definitions(planner_node_errors_test PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
ament_add_ros_isolated_gtest(planner_client_test planner_client_test.cpp TIMEOUT 90)
target_link_libraries(planner_client_test ${PROJECT_NAME} ${lifecycle_msgs_TARGETS})
target_compile_definitions(planner_client_test PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
ament_add_ros_isolated_gtest(kairos_planner_node_test kairos_planner_node_test.cpp TIMEOUT 60)
target_link_libraries(kairos_planner_node_test
  ${PROJECT_NAME}
  ament_index_cpp::ament_index_cpp
  ${lifecycle_msgs_TARGETS}
)
target_compile_definitions(kairos_planner_node_test PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

# Kairos, POPF and a silent solver on the bank; Kairos validates the plans
find_package(kairos REQUIRED)
ament_add_ros_isolated_gtest(planner_bank_test planner_bank_test.cpp TIMEOUT 240)
target_link_libraries(planner_bank_test
  ${PROJECT_NAME}
  ament_index_cpp::ament_index_cpp
  kairos::kairos
  ${lifecycle_msgs_TARGETS}
)
target_compile_definitions(planner_bank_test PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

# Tests discover only nodes on this host: parallel jobs do not mix
get_property(_tests DIRECTORY PROPERTY TESTS)
if(_tests)
  set_tests_properties(${_tests}
    PROPERTIES ENVIRONMENT "ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST")
endif()
