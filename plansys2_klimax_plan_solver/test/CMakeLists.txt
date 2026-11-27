install(DIRECTORY
  ${CMAKE_CURRENT_SOURCE_DIR}/hddl
  DESTINATION share/${PROJECT_NAME}
)

add_subdirectory(unit)
