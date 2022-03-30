ament_add_gtest(pddl_parser_test pddl_parser_test.cpp TIMEOUT 120)
ament_target_dependencies(pddl_parser_test ${dependencies})
target_link_libraries(pddl_parser_test plansys2_pddl_parser)

install(DIRECTORY
  pddl
  DESTINATION share/${PROJECT_NAME}
)
