ament_add_gtest(executor_test executor_test.cpp)
target_link_libraries(executor_test ${PROJECT_NAME})

ament_add_gtest(executor_client_test
  executor_client_test.cpp
  behavior_tree/OpenGripper.cpp
  behavior_tree/CloseGripper.cpp
  behavior_tree/Move.cpp
  behavior_tree/RepeatServer.cpp
)
ament_target_dependencies(executor_client_test ${dependencies} test_msgs)
target_link_libraries(executor_client_test ${PROJECT_NAME})

install(FILES
  behavior_tree/assemble.xml
  behavior_tree/transport.xml
  DESTINATION share/${PROJECT_NAME}
)

