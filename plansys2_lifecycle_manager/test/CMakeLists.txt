ament_add_gtest(lf_manager_test lf_manager_test.cpp)
ament_target_dependencies(lf_manager_test ${dependencies})
target_link_libraries(lf_manager_test ${PROJECT_NAME})
