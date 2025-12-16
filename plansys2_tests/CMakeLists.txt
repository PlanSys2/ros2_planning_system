cmake_minimum_required(VERSION 3.5)

project(plansys2_tests)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(rclcpp REQUIRED)

set(${PROJECT_NAME}_SOURCES
  src/plansys2_tests/test_action_node.cpp
  src/plansys2_tests/execution_logger.cpp
)

add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_SOURCES})
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(${PROJECT_NAME}
  PUBLIC
  ${plansys2_msgs_TARGETS}
  plansys2_executor::plansys2_executor
  rclcpp::rclcpp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_index_cpp REQUIRED)
  find_package(plansys2_domain_expert REQUIRED)
  find_package(plansys2_problem_expert REQUIRED)
  find_package(plansys2_planner REQUIRED)
  find_package(plansys2_pddl_parser REQUIRED)

  add_subdirectory(test_1)
  add_subdirectory(test_2)
  add_subdirectory(test_3)
  add_subdirectory(test_4)
endif()

ament_package()
