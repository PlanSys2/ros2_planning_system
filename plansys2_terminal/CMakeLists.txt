cmake_minimum_required(VERSION 3.5)

project(plansys2_terminal)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_domain_expert REQUIRED)
find_package(plansys2_problem_expert REQUIRED)
find_package(plansys2_planner REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(plansys2_pddl_parser REQUIRED)

add_library(terminal SHARED
  src/plansys2_terminal/Terminal.cpp
)
target_include_directories(terminal PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(terminal
  PUBLIC
  plansys2_domain_expert::plansys2_domain_expert
  plansys2_problem_expert::plansys2_problem_expert
  plansys2_planner::plansys2_planner
  plansys2_executor::plansys2_executor
  rclcpp::rclcpp
  PRIVATE
  plansys2_pddl_parser::plansys2_pddl_parser
  ${plansys2_msgs_TARGETS}
  readline
)
target_compile_definitions(terminal PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

add_executable(${PROJECT_NAME}
  src/terminal_node.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(${PROJECT_NAME}
  PRIVATE
  rclcpp::rclcpp
  terminal
)

install(TARGETS terminal
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_index_cpp REQUIRED)
  add_subdirectory(test)
endif()

ament_package()
