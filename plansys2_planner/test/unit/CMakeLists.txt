ament_add_gtest(planner_test planner_test.cpp)
target_link_libraries(planner_test ${PROJECT_NAME} dl)
target_compile_definitions(planner_test PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")