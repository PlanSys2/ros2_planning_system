ament_add_ros_isolated_gtest(domain_expert_test domain_expert_test.cpp TIMEOUT 60)
target_link_libraries(domain_expert_test
  ${PROJECT_NAME}
  ament_index_cpp::ament_index_cpp
  ${lifecycle_msgs_TARGETS}
)

ament_add_ros_isolated_gtest(domain_expert_node_test domain_expert_node_test.cpp TIMEOUT 60)
target_link_libraries(domain_expert_node_test
  ${PROJECT_NAME}
  ament_index_cpp::ament_index_cpp
)

ament_add_ros_isolated_gtest(domain_reader_test domain_reader_test.cpp TIMEOUT 60)
target_link_libraries(domain_reader_test
  ${PROJECT_NAME}
  ament_index_cpp::ament_index_cpp
)

ament_add_ros_isolated_gtest(types_test types_test.cpp TIMEOUT 60)
target_link_libraries(types_test
  ${PROJECT_NAME}
  ament_index_cpp::ament_index_cpp
)
# Tests discover only nodes on this host: parallel jobs do not mix
get_property(_tests DIRECTORY PROPERTY TESTS)
if(_tests)
  set_tests_properties(${_tests}
    PROPERTIES ENVIRONMENT "ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST")
endif()
