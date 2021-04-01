install(DIRECTORY pddl
  DESTINATION share/${PROJECT_NAME}/test_1
)

ament_add_gtest(test_1 test_1.cpp)
target_link_libraries(test_1 ${PROJECT_NAME})
