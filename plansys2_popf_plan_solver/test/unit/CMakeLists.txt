# Kairos validates the plans of the bank
find_package(kairos REQUIRED)
ament_add_ros_isolated_gtest(popf_test popf_test.cpp TIMEOUT 120)
target_link_libraries(popf_test
  kairos::kairos
  ${PROJECT_NAME}
  ament_index_cpp::ament_index_cpp
  dl
  pluginlib::pluginlib
)
target_compile_definitions(popf_test PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
# Tests discover only nodes on this host: parallel jobs do not mix
get_property(_tests DIRECTORY PROPERTY TESTS)
if(_tests)
  set_tests_properties(${_tests}
    PROPERTIES ENVIRONMENT "ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST")
endif()
