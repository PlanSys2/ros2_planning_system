cmake_minimum_required(VERSION 3.5)

project(plansys2_problem_expert)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(plansys2_pddl_parser REQUIRED)
find_package(plansys2_core REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_domain_expert REQUIRED)
find_package(std_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)


set(dependencies
    rclcpp::rclcpp
    rclcpp_action::rclcpp_action
    rclcpp_lifecycle::rclcpp_lifecycle
    plansys2_pddl_parser::plansys2_pddl_parser
    ament_index_cpp::ament_index_cpp
    plansys2_core::plansys2_core
    plansys2_domain_expert::plansys2_domain_expert
    ${plansys2_msgs_TARGETS}
    ${std_msgs_TARGETS}
    ${lifecycle_msgs_TARGETS}
)

set(package_dependencies
    rclcpp
    rclcpp_action
    rclcpp_lifecycle
    plansys2_pddl_parser
    ament_index_cpp
    plansys2_core
    plansys2_msgs
    plansys2_domain_expert
    std_msgs
    lifecycle_msgs
)

include_directories(include)

set(PROBLEM_EXPERT_SOURCES
  src/plansys2_problem_expert/ProblemExpert.cpp
  src/plansys2_problem_expert/ProblemExpertClient.cpp
  src/plansys2_problem_expert/ProblemExpertNode.cpp
  src/plansys2_problem_expert/Utils.cpp
)

add_library(${PROJECT_NAME} SHARED ${PROBLEM_EXPERT_SOURCES})
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${PROJECT_NAME} PUBLIC ${dependencies})

add_executable(problem_expert_node
  src/problem_expert_node.cpp
)
target_link_libraries(problem_expert_node PUBLIC ${PROJECT_NAME} ${dependencies})

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  problem_expert_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(${package_dependencies})

ament_package()
