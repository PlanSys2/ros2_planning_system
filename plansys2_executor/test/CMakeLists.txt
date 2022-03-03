set(TEST_PDDL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/pddl)
set(TEST_BEHAVIOR_TREES_DIR ${CMAKE_CURRENT_SOURCE_DIR}/test_behavior_trees)
set(TEST_DATA_DIR ${CMAKE_CURRENT_SOURCE_DIR}/test_data)
set(TEST_LAUNCH_DIR ${CMAKE_CURRENT_SOURCE_DIR}/test_launch_files)

install(DIRECTORY
  ${TEST_PDDL_DIR}
  ${TEST_BEHAVIOR_TREES_DIR}
  ${TEST_DATA_DIR}
  DESTINATION share/${PROJECT_NAME}
)

add_subdirectory(unit)
#add_subdirectory(integration)
