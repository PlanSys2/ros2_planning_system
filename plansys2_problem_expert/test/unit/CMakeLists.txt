# The bank, through the experts: what reaches the planner is what was loaded
find_package(kairos REQUIRED)
ament_add_gtest(bank_roundtrip_test bank_roundtrip_test.cpp TIMEOUT 120)
target_link_libraries(bank_roundtrip_test
  ${PROJECT_NAME}
  ament_index_cpp::ament_index_cpp
  kairos::kairos
)

ament_add_ros_isolated_gtest(utils_test utils_test.cpp TIMEOUT 60)
target_link_libraries(utils_test
  ${PROJECT_NAME}
  ament_index_cpp::ament_index_cpp
)

ament_add_ros_isolated_gtest(client_no_answer_test client_no_answer_test.cpp TIMEOUT 180)
target_link_libraries(client_no_answer_test
  ${PROJECT_NAME}
)

ament_add_ros_isolated_gtest(check_in_expert_test check_in_expert_test.cpp TIMEOUT 60)
target_link_libraries(check_in_expert_test
  ${PROJECT_NAME}
  ament_index_cpp::ament_index_cpp
)

ament_add_gtest_executable(domain_change_test domain_change_test.cpp)
target_link_libraries(domain_change_test
  ${PROJECT_NAME}
  ament_index_cpp::ament_index_cpp
)
foreach(scenario
    DomainChangeTest.new_domain_works_as_soon_as_the_change_returns
    DomainChangeTest.problem_expert_configured_late_takes_the_current_domain
    DomainChangeTest.same_domain_again_keeps_the_problem
    DomainChangeRejectedTest.domain_stays_if_the_problem_expert_rejects_it)
  string(REGEX REPLACE ".*\\." "" name ${scenario})
  ament_add_ros_isolated_gtest_test(domain_change_test
    TEST_NAME domain_change_${name}
    ENV GTEST_FILTER=${scenario}
    TIMEOUT 60)
endforeach()

ament_add_ros_isolated_gtest(effect_undo_test effect_undo_test.cpp TIMEOUT 60)
target_link_libraries(effect_undo_test
  ${PROJECT_NAME}
  ament_index_cpp::ament_index_cpp
)

ament_add_gtest(derived_test derived_test.cpp TIMEOUT 60)
target_link_libraries(derived_test ${PROJECT_NAME})

ament_add_ros_isolated_gtest(object_fluents_test object_fluents_test.cpp TIMEOUT 60)
target_link_libraries(object_fluents_test ${PROJECT_NAME})

ament_add_ros_isolated_gtest(dynamics_test dynamics_test.cpp TIMEOUT 60)
target_link_libraries(dynamics_test ${PROJECT_NAME})

ament_add_ros_isolated_gtest(timed_facts_test timed_facts_test.cpp TIMEOUT 60)
target_link_libraries(timed_facts_test
  ${PROJECT_NAME}
  ament_index_cpp::ament_index_cpp
)

ament_add_ros_isolated_gtest(problem_expert_test problem_expert_test.cpp TIMEOUT 60)
target_link_libraries(problem_expert_test
  ${PROJECT_NAME}
  ament_index_cpp::ament_index_cpp
)

ament_add_ros_isolated_gtest(problem_expert_node_test problem_expert_node_test.cpp TIMEOUT 60)
target_link_libraries(problem_expert_node_test
  ${PROJECT_NAME}
  ament_index_cpp::ament_index_cpp
)

ament_add_ros_isolated_gtest(quantifier_test quantifier_test.cpp TIMEOUT 60)
target_link_libraries(quantifier_test
  ${PROJECT_NAME}
  ament_index_cpp::ament_index_cpp
)

ament_add_ros_isolated_gtest(problem_expert_misc_test problem_expert_misc_test.cpp TIMEOUT 60)
target_link_libraries(problem_expert_misc_test
  ${PROJECT_NAME}
  ament_index_cpp::ament_index_cpp
)

ament_add_ros_isolated_gtest(malformed_pddl_test malformed_pddl_test.cpp TIMEOUT 60)
target_link_libraries(malformed_pddl_test
  ${PROJECT_NAME}
  ament_index_cpp::ament_index_cpp
)

ament_add_ros_isolated_gtest(tree_node_ids_test tree_node_ids_test.cpp TIMEOUT 60)
target_link_libraries(tree_node_ids_test
  ${PROJECT_NAME}
  ament_index_cpp::ament_index_cpp
)

# Tests discover only nodes on this host: parallel jobs do not mix
get_property(_tests DIRECTORY PROPERTY TESTS)
if(_tests)
  set_tests_properties(${_tests}
    PROPERTIES ENVIRONMENT "ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST")
endif()
