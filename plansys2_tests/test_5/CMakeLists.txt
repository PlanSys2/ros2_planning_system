install(DIRECTORY pddl
  DESTINATION share/${PROJECT_NAME}/test_5
)

ament_add_gtest(test_5 test_5.cpp)
target_link_libraries(test_5 ${PROJECT_NAME})
