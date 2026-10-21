ament_add_ros_isolated_gtest(lf_manager_test lf_manager_test.cpp TIMEOUT 90)
target_link_libraries(lf_manager_test
  ${PROJECT_NAME}
  rclcpp_lifecycle::rclcpp_lifecycle
  plansys2_core::plansys2_core
)

ament_add_ros_isolated_gtest(lf_supervisor_test lf_supervisor_test.cpp TIMEOUT 180)
target_link_libraries(lf_supervisor_test
  ${PROJECT_NAME}
  rclcpp_lifecycle::rclcpp_lifecycle
  plansys2_core::plansys2_core
)

ament_add_ros_isolated_gtest(lf_manager_shutdown_test lf_manager_shutdown_test.cpp TIMEOUT 60)
target_link_libraries(lf_manager_shutdown_test
  ${PROJECT_NAME}
  rclcpp_lifecycle::rclcpp_lifecycle
  plansys2_core::plansys2_core
)

# Tests discover only nodes on this host: parallel jobs do not mix
get_property(_tests DIRECTORY PROPERTY TESTS)
if(_tests)
  set_tests_properties(${_tests}
    PROPERTIES ENVIRONMENT "ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST")
endif()
