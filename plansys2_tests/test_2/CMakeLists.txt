install(DIRECTORY pddl
  DESTINATION share/${PROJECT_NAME}/test_2
)

ament_add_gtest(test_2 test_2.cpp)
target_link_libraries(test_2 ${PROJECT_NAME})
