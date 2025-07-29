cmake_minimum_required(VERSION 3.5)

project(plansys2_planner)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(plansys2_core REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_domain_expert REQUIRED)
find_package(plansys2_problem_expert REQUIRED)
find_package(plansys2_popf_plan_solver REQUIRED)
find_package(pluginlib REQUIRED)
find_package(lifecycle_msgs REQUIRED)

set(PLANNER_SOURCES
  src/plansys2_planner/PlannerClient.cpp
  src/plansys2_planner/PlannerNode.cpp
)

add_library(${PROJECT_NAME} SHARED ${PLANNER_SOURCES})
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${PROJECT_NAME}
  PUBLIC
  ${lifecycle_msgs_TARGETS}
  plansys2_core::plansys2_core
  plansys2_domain_expert::plansys2_domain_expert
  plansys2_problem_expert::plansys2_problem_expert
  ${plansys2_msgs_TARGETS}
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  PRIVATE
  plansys2_popf_plan_solver::plansys2_popf_plan_solver
)
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

add_executable(planner_node
  src/planner_node.cpp
)
target_include_directories(planner_node PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_compile_definitions(planner_node PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
target_link_libraries(planner_node
  PRIVATE
  ${PROJECT_NAME}
  rclcpp::rclcpp
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS planner_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_index_cpp REQUIRED)
  find_package(plansys2_pddl_parser REQUIRED)
  add_subdirectory(test)
endif()

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  plansys2_core
  plansys2_pddl_parser
  ament_index_cpp
  plansys2_msgs
  plansys2_domain_expert
  plansys2_problem_expert
  plansys2_popf_plan_solver
  pluginlib
  lifecycle_msgs
)

ament_package()
