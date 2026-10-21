cmake_minimum_required(VERSION 3.16)

project(plansys2_kairos_plan_solver)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(plansys2_core REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(kairos REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/plansys2_kairos_plan_solver/kairos_plan_solver.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${PROJECT_NAME}
  PUBLIC
  plansys2_core::plansys2_core
  ${plansys2_msgs_TARGETS}
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  PRIVATE
  kairos::kairos
  pluginlib::pluginlib
)

target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
# The kairos executable for the isolated mode, installed with the library
get_filename_component(KAIROS_PREFIX "${kairos_DIR}/../../.." ABSOLUTE)
target_compile_definitions(${PROJECT_NAME} PRIVATE "KAIROS_EXECUTABLE=\"${KAIROS_PREFIX}/bin/kairos\"")

pluginlib_export_plugin_description_file(plansys2_core plansys2_kairos_plan_solver_plugin.xml)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(FILES plansys2_kairos_plan_solver_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}
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
  find_package(ament_cmake_ros REQUIRED)
  find_package(ament_index_cpp REQUIRED)
  add_subdirectory(test)
endif()

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  plansys2_core
  plansys2_msgs
  pluginlib
)

ament_package()
