install(DIRECTORY pddl
  DESTINATION share/${PROJECT_NAME}/test_1
)

ament_add_ros_isolated_gtest(test_1 test_1.cpp TIMEOUT 70)
target_link_libraries(test_1
  ${PROJECT_NAME}
  ament_index_cpp::ament_index_cpp
  plansys2_domain_expert::plansys2_domain_expert
  plansys2_problem_expert::plansys2_problem_expert
  plansys2_planner::plansys2_planner
)

# Tests discover only nodes on this host: parallel jobs do not mix
get_property(_tests DIRECTORY PROPERTY TESTS)
if(_tests)
  set_tests_properties(${_tests}
    PROPERTIES ENVIRONMENT "ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST")
endif()
