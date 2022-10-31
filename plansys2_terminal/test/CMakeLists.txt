ament_add_gtest(terminal_test terminal_test.cpp TIMEOUT 500)
ament_target_dependencies(terminal_test ${dependencies})
target_link_libraries(terminal_test terminal readline)

install(DIRECTORY
  pddl
  DESTINATION share/${PROJECT_NAME}
)
