install(DIRECTORY pddl
  DESTINATION share/${PROJECT_NAME}/test_4
)

ament_add_gtest(test_4 test_4.cpp)
target_link_libraries(test_4 ${PROJECT_NAME})
