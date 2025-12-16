cmake_minimum_required(VERSION 3.5)

project(plansys2_domain_expert)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(plansys2_pddl_parser REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_core REQUIRED)
find_package(plansys2_popf_plan_solver REQUIRED)
find_package(std_msgs REQUIRED)

set(DOMAIN_EXPERT_SOURCES
  src/plansys2_domain_expert/DomainExpert.cpp
  src/plansys2_domain_expert/DomainExpertClient.cpp
  src/plansys2_domain_expert/DomainReader.cpp
  src/plansys2_domain_expert/DomainExpertNode.cpp
)

add_library(${PROJECT_NAME} SHARED ${DOMAIN_EXPERT_SOURCES})
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${PROJECT_NAME}
  PUBLIC
  plansys2_core::plansys2_core
  ${plansys2_msgs_TARGETS}
  plansys2_pddl_parser::plansys2_pddl_parser
  plansys2_popf_plan_solver::plansys2_popf_plan_solver
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${std_msgs_TARGETS}
)

add_executable(domain_expert_node
  src/domain_expert_node.cpp
)
target_include_directories(domain_expert_node PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(domain_expert_node
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

install(TARGETS domain_expert_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_index_cpp REQUIRED)
  find_package(lifecycle_msgs REQUIRED)
  add_subdirectory(test)
endif()

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  plansys2_pddl_parser
  ament_index_cpp
  plansys2_msgs
  plansys2_core
  plansys2_popf_plan_solver
  std_msgs
  lifecycle_msgs
)

ament_package()
