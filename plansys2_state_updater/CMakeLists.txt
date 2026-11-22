cmake_minimum_required(VERSION 3.16)

project(plansys2_state_updater)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rcpputils REQUIRED)
find_package(rosidl_runtime_cpp REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(rosidl_typesupport_introspection_cpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_pddl_parser REQUIRED)
find_package(plansys2_domain_expert REQUIRED)
find_package(plansys2_problem_expert REQUIRED)

# The rules engine, without ROS nodes
add_library(${PROJECT_NAME}_core SHARED
  src/value.cpp
  src/paths.cpp
  src/transforms.cpp
  src/rules.cpp
  src/instances.cpp
  src/domain_check.cpp
  src/engine.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${PROJECT_NAME}_core
  PUBLIC
  ${plansys2_msgs_TARGETS}
  pluginlib::pluginlib
  yaml-cpp::yaml-cpp
)

# The RulesUpdater problem updater plugin
add_library(${PROJECT_NAME} SHARED
  src/message_reader.cpp
  src/problem_writer.cpp
  src/domain_reader.cpp
  src/rules_updater.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${PROJECT_NAME}
  PUBLIC
  ${PROJECT_NAME}_core
  ${rcl_interfaces_TARGETS}
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
  plansys2_domain_expert::plansys2_domain_expert
  plansys2_pddl_parser::plansys2_pddl_parser
  plansys2_problem_expert::plansys2_problem_expert
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  rcpputils::rcpputils
  rosidl_runtime_cpp::rosidl_runtime_cpp
  rosidl_typesupport_cpp::rosidl_typesupport_cpp
  rosidl_typesupport_introspection_cpp::rosidl_typesupport_introspection_cpp
)

add_library(${PROJECT_NAME}_plugins SHARED src/plugins/min_of.cpp)
target_link_libraries(${PROJECT_NAME}_plugins PRIVATE ${PROJECT_NAME}_core)
pluginlib_export_plugin_description_file(${PROJECT_NAME} plugins.xml)
pluginlib_export_plugin_description_file(plansys2_problem_expert updater_plugins.xml)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS examples/replanning_mission.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}_core ${PROJECT_NAME} ${PROJECT_NAME}_plugins
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Humble's uncrustify (0.72) formats otherwise than the newer one this code follows
  if("$ENV{ROS_DISTRO}" STREQUAL "humble")
    set(ament_cmake_uncrustify_FOUND TRUE)
  endif()
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test/unit)

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(state_updater_live test/test_state_updater_live.py TIMEOUT 600)
  ament_add_pytest_test(state_updater_mission test/test_state_updater_mission.py TIMEOUT 300)
endif()

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(${PROJECT_NAME}_core ${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  plansys2_domain_expert
  plansys2_msgs
  plansys2_pddl_parser
  plansys2_problem_expert
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rcpputils
  rosidl_typesupport_introspection_cpp
  std_srvs
  yaml_cpp_vendor
  yaml-cpp
)

ament_package()
