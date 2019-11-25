ament_add_gtest(domain_expert_test domain_expert_test.cpp)
target_link_libraries(domain_expert_test ${PROJECT_NAME})

ament_add_gtest(types_test types_test.cpp)
target_link_libraries(types_test ${PROJECT_NAME})