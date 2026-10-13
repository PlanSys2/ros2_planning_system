ament_add_gtest(kairos_test kairos_test.cpp TIMEOUT 300)
target_link_libraries(kairos_test
  ${PROJECT_NAME}
  ament_index_cpp::ament_index_cpp
  pluginlib::pluginlib
  kairos::kairos
)
target_compile_definitions(kairos_test PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
