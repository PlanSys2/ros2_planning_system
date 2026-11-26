foreach(test value_paths rules instances domain_check engine input)
  ament_add_gtest(${test}_test ${test}_test.cpp)
  target_link_libraries(${test}_test ${PROJECT_NAME}_core)
endforeach()

find_package(diagnostic_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
ament_add_gtest(message_reader_test message_reader_test.cpp)
target_link_libraries(message_reader_test
  ${PROJECT_NAME} ${diagnostic_msgs_TARGETS} ${sensor_msgs_TARGETS})

ament_add_ros_isolated_gtest(rules_updater_test rules_updater_test.cpp TIMEOUT 120)
target_compile_definitions(rules_updater_test PRIVATE CONFIG_DIR="${PROJECT_SOURCE_DIR}/config")
find_package(rosgraph_msgs REQUIRED)
target_link_libraries(rules_updater_test
  ${PROJECT_NAME} ${diagnostic_msgs_TARGETS} ${rosgraph_msgs_TARGETS} ${std_msgs_TARGETS}
  ${std_srvs_TARGETS})
