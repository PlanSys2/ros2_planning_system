add_library(plansys2_close_gripper_bt_node SHARED
  ../behavior_tree/CloseGripper.cpp
)
list(APPEND plugin_libs plansys2_close_gripper_bt_node)

add_library(plansys2_open_gripper_bt_node SHARED
  ../behavior_tree/OpenGripper.cpp
)
list(APPEND plugin_libs plansys2_open_gripper_bt_node)

add_library(plansys2_move_bt_test_node SHARED ../behavior_tree/Move.cpp)
list(APPEND plugin_libs plansys2_move_bt_test_node)

add_library(plansys2_failure_test_nodes SHARED
  ../behavior_tree/FailureNodes.cpp
)
list(APPEND plugin_libs plansys2_failure_test_nodes)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

ament_add_gtest(bt_action_test bt_action_test.cpp)
target_link_libraries(bt_action_test
  ${PROJECT_NAME}
)
ament_target_dependencies(bt_action_test ${dependencies})

install(DIRECTORY ../behavior_tree
  DESTINATION share/${PROJECT_NAME}/test
)

install(TARGETS
  ${plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
