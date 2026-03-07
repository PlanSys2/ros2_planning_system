cmake_minimum_required(VERSION 3.5)

project(plansys2_core)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(plansys2_pddl_parser REQUIRED)
find_package(plansys2_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/plansys2_core/Action.cpp
  src/plansys2_core/DerivedResolutionGraph.cpp
  src/plansys2_core/PlanSolverBase.cpp
  src/plansys2_core/State.cpp
  src/plansys2_core/Utils.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${PROJECT_NAME}
  PUBLIC
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  plansys2_pddl_parser::plansys2_pddl_parser
  ${plansys2_msgs_TARGETS}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_index_cpp REQUIRED)
  add_subdirectory(test)
endif()

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  plansys2_pddl_parser
  plansys2_msgs
)

ament_package()
