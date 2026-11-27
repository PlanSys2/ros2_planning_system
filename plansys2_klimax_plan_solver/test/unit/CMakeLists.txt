ament_add_ros_isolated_gtest(klimax_test klimax_test.cpp TIMEOUT 60)
target_link_libraries(klimax_test
  ${PROJECT_NAME}
  ament_index_cpp::ament_index_cpp
  pluginlib::pluginlib
  klimax::klimax
)
target_compile_definitions(klimax_test PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

# Tests discover only nodes on this host: parallel jobs do not mix
get_property(_tests DIRECTORY PROPERTY TESTS)
if(_tests)
  set_tests_properties(${_tests}
    PROPERTIES ENVIRONMENT "ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST")
endif()
