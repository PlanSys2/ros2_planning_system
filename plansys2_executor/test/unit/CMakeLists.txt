ament_add_gtest(executor_test executor_test.cpp)
target_link_libraries(executor_test ${PROJECT_NAME})

ament_add_gtest(executor_client_test
  executor_client_test.cpp
)
ament_target_dependencies(executor_client_test ${dependencies} test_msgs)
target_link_libraries(executor_client_test ${PROJECT_NAME})

