cmake_minimum_required(VERSION 3.5)
project(plansys2_executor)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp_cascade_lifecycle REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(plansys2_core REQUIRED)
find_package(plansys2_pddl_parser REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_domain_expert REQUIRED)
find_package(plansys2_problem_expert REQUIRED)
find_package(plansys2_planner REQUIRED)
find_package(pluginlib REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

set(EXECUTOR_SOURCES
  src/plansys2_executor/ExecutorClient.cpp
  src/plansys2_executor/ActionExecutor.cpp
  src/plansys2_executor/ActionExecutorClient.cpp
  src/plansys2_executor/ExecutorNode.cpp
  src/plansys2_executor/ComputeBT.cpp
  src/plansys2_executor/behavior_tree/execute_action_node.cpp
  src/plansys2_executor/behavior_tree/wait_action_node.cpp
  src/plansys2_executor/behavior_tree/check_action_node.cpp
  src/plansys2_executor/behavior_tree/wait_atstart_req_node.cpp
  src/plansys2_executor/behavior_tree/check_overall_req_node.cpp
  src/plansys2_executor/behavior_tree/check_atend_req_node.cpp
  src/plansys2_executor/behavior_tree/check_timeout_node.cpp
  src/plansys2_executor/behavior_tree/apply_atstart_effect_node.cpp
  src/plansys2_executor/behavior_tree/restore_atstart_effect_node.cpp
  src/plansys2_executor/behavior_tree/apply_atend_effect_node.cpp
)

add_library(${PROJECT_NAME} SHARED ${EXECUTOR_SOURCES})
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${PROJECT_NAME}
  PUBLIC
  behaviortree_cpp::behaviortree_cpp
  ${lifecycle_msgs_TARGETS}
  ${plansys2_msgs_TARGETS}
  plansys2_core::plansys2_core
  plansys2_domain_expert::plansys2_domain_expert
  plansys2_pddl_parser::plansys2_pddl_parser
  plansys2_planner::plansys2_planner
  plansys2_problem_expert::plansys2_problem_expert
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  rclcpp_cascade_lifecycle::rclcpp_cascade_lifecycle
  rclcpp_lifecycle::rclcpp_lifecycle
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
  PRIVATE
  ament_index_cpp::ament_index_cpp
)

add_executable(executor_node
  src/executor_node.cpp
)
target_include_directories(executor_node PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(executor_node
  PRIVATE
  ${PROJECT_NAME}
  rclcpp::rclcpp
)

add_executable(compute_bt
  src/compute_bt.cpp
)
target_link_libraries(compute_bt
  PRIVATE
  ${PROJECT_NAME}
  rclcpp::rclcpp
)

add_library(bt_builder_plugins SHARED
  src/plansys2_executor/bt_builder_plugins/simple_bt_builder.cpp
  src/plansys2_executor/bt_builder_plugins/stn_bt_builder.cpp
  src/plansys2_executor/bt_builder_plugins/sequential_bt_builder.cpp
)
target_include_directories(bt_builder_plugins PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(bt_builder_plugins
  PUBLIC
  Eigen3::Eigen
  ${plansys2_msgs_TARGETS}
  plansys2_core::plansys2_core
  plansys2_domain_expert::plansys2_domain_expert
  plansys2_problem_expert::plansys2_problem_expert
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  ${std_msgs_TARGETS}
  PRIVATE
  behaviortree_cpp::behaviortree_cpp
  plansys2_pddl_parser::plansys2_pddl_parser
  pluginlib::pluginlib
)

pluginlib_export_plugin_description_file(plansys2_executor plugins.xml)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  DIRECTORY launch behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS bt_builder_plugins
  EXPORT bt_builder_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS bt_builder_plugins executor_node compute_bt
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  add_subdirectory(test)
endif()

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(${PROJECT_NAME} bt_builder_plugins)
ament_export_targets(export_${PROJECT_NAME} bt_builder_plugins)
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  lifecycle_msgs
  rclcpp_cascade_lifecycle
  rclcpp_action
  plansys2_pddl_parser
  ament_index_cpp
  plansys2_core
  plansys2_msgs
  plansys2_domain_expert
  plansys2_problem_expert
  plansys2_planner
  pluginlib
  behaviortree_cpp
  std_msgs
  std_srvs
)
ament_package()
