cmake_minimum_required(VERSION 3.5)

project(plansys2_bt_actions)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(BT_ACTIONS_SOURCES
  src/plansys2_bt_actions/BTAction.cpp
)

add_library(${PROJECT_NAME} SHARED ${BT_ACTIONS_SOURCES})
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${PROJECT_NAME}
  PUBLIC
  behaviortree_cpp::behaviortree_cpp
  plansys2_executor::plansys2_executor
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  rclcpp_lifecycle::rclcpp_lifecycle
  PRIVATE
  ${std_msgs_TARGETS}
)

add_executable(bt_action_node
  src/bt_action_node.cpp
)
target_include_directories(bt_action_node PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(bt_action_node
  PRIVATE
  ${PROJECT_NAME}
  rclcpp::rclcpp
  ${lifecycle_msgs_TARGETS}
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

install(TARGETS bt_action_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(plansys2_msgs REQUIRED)
  find_package(test_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(tf2_geometry_msgs)
  add_subdirectory(test)
endif()

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  plansys2_executor
  behaviortree_cpp
)

ament_package()