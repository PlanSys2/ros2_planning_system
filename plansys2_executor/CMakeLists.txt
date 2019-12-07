cmake_minimum_required(VERSION 3.5)
project(plansys2_executor)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE DEBUG)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(pddl_parser REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_domain_expert REQUIRED)
find_package(plansys2_problem_expert REQUIRED)
find_package(plansys2_planner REQUIRED)

set(dependencies
    rclcpp
    rclcpp_lifecycle
    rclcpp_action
    pddl_parser
    ament_index_cpp
    plansys2_msgs
    plansys2_domain_expert
    plansys2_problem_expert
    plansys2_planner
)

include_directories(include)

set(EXECUTOR_SOURCES
  src/plansys2_executor/ExecutorClient.cpp
  src/plansys2_executor/ActionExecutor.cpp
)

add_library(${PROJECT_NAME} SHARED ${EXECUTOR_SOURCES})
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(executor_node
  src/executor_node.cpp
  src/plansys2_executor/ExecutorNode.cpp)
ament_target_dependencies(executor_node ${dependencies})
target_link_libraries(executor_node ${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}
  executor_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()
