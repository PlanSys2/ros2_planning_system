project(plansys2_terminal)

cmake_minimum_required(VERSION 3.5)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE DEBUG)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_domain_expert REQUIRED)
find_package(plansys2_problem_expert REQUIRED)
find_package(plansys2_planner REQUIRED)
find_package(plansys2_executor REQUIRED)

set(dependencies
    rclcpp
    rclcpp_action
    rclcpp_lifecycle
    ament_index_cpp
    plansys2_msgs
    plansys2_domain_expert
    plansys2_problem_expert
    plansys2_planner
    plansys2_executor
)

add_executable(${PROJECT_NAME}
  src/terminal_node.cpp)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_link_libraries(${PROJECT_NAME} readline)

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()