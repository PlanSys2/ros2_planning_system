install(DIRECTORY
  ${CMAKE_CURRENT_SOURCE_DIR}/pddl
  DESTINATION share/${PROJECT_NAME}
)

add_subdirectory(unit)
