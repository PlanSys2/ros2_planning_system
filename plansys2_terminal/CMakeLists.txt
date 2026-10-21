cmake_minimum_required(VERSION 3.16)

project(plansys2_terminal)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra)
endif()

set(CMAKE_CXX_STANDARD 20)
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
# libplansys2_terminal.so, not a generic libterminal.so in lib/
set_target_properties(terminal PROPERTIES OUTPUT_NAME plansys2_terminal)

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
  # Humble's uncrustify (0.72) formats otherwise than the newer one this code follows
  if("$ENV{ROS_DISTRO}" STREQUAL "humble")
    set(ament_cmake_uncrustify_FOUND TRUE)
  endif()
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_ros REQUIRED)
  find_package(ament_index_cpp REQUIRED)
  add_subdirectory(test)
endif()

ament_package()
