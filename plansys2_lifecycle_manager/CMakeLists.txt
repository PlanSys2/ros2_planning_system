project(plansys2_lifecycle_manager)

cmake_minimum_required(VERSION 3.5)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)

set(dependencies
    rclcpp
    rclcpp_lifecycle
    lifecycle_msgs
)

include_directories(include)

add_library(${PROJECT_NAME}
  src/plansys2_lifecycle_manager/lifecycle_manager.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})


add_executable(lifecycle_manager_node
  src/lifecycle_manager_node.cpp)
ament_target_dependencies(lifecycle_manager_node ${dependencies})
target_link_libraries(lifecycle_manager_node ${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/
)

install(TARGETS
  ${PROJECT_NAME}
  lifecycle_manager_node
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