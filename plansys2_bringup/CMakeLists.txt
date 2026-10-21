cmake_minimum_required(VERSION 3.16)

project(plansys2_bringup)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(plansys2_domain_expert REQUIRED)
find_package(plansys2_problem_expert REQUIRED)
find_package(plansys2_planner REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)

add_executable(plansys2_node
  src/plansys2_node.cpp
)
target_link_libraries(plansys2_node
  PRIVATE
  rclcpp::rclcpp
  plansys2_domain_expert::plansys2_domain_expert
  plansys2_problem_expert::plansys2_problem_expert
  plansys2_planner::plansys2_planner
  plansys2_executor::plansys2_executor
  rclcpp_lifecycle::rclcpp_lifecycle
  ${lifecycle_msgs_TARGETS}
)
target_compile_definitions(plansys2_node PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Humble's uncrustify (0.72) formats otherwise than the newer one this code follows
  if("$ENV{ROS_DISTRO}" STREQUAL "humble")
    set(ament_cmake_uncrustify_FOUND TRUE)
  endif()
  ament_lint_auto_find_test_dependencies()

  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_monolithic_responsive.py TIMEOUT 180)
  add_launch_test(test/test_monolithic_real_time.py TIMEOUT 60)
  add_launch_test(test/test_bringup_problem_file.py TIMEOUT 300)
  add_launch_test(test/test_distributed_bt_files.py TIMEOUT 120)
endif()

install(DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}
)
install(TARGETS plansys2_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(
  rclcpp
  plansys2_domain_expert
  plansys2_problem_expert
  plansys2_planner
  plansys2_executor
  rclcpp_lifecycle
  lifecycle_msgs)
ament_package()
