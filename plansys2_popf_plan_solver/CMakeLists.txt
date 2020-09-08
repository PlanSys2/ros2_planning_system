project(plansys2_popf_plan_solver)

cmake_minimum_required(VERSION 3.5)

find_package(Threads REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(plansys2_core REQUIRED)
find_package(pluginlib REQUIRED)

set(dependencies
  rclcpp
  ament_index_cpp
  plansys2_core
  pluginlib
)

include_directories(
  include
)

add_library(${PROJECT_NAME} SHARED
  src/plansys2_popf_plan_solver/popf_plan_solver.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
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

pluginlib_export_plugin_description_file(plansys2_core plansys2_popf_plan_solver_plugin.xml)

ament_package()