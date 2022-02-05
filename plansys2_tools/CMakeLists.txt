project(plansys2_tools)
cmake_minimum_required(VERSION 3.5)

find_package(Threads REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(qt_gui_cpp REQUIRED)
find_package(rqt_gui_cpp REQUIRED)
find_package(Qt5Widgets REQUIRED)

set(CMAKE_CXX_STANDARD 17)

set(dependencies
  rclcpp
  plansys2_msgs
  qt_gui_cpp
  rqt_gui_cpp
)

include_directories(include)

add_library(plansys2_logger SHARED src/plansys2_logger/LoggerNode.cpp)
ament_target_dependencies(plansys2_logger ${dependencies})

add_executable(logger src/logger.cpp)
ament_target_dependencies(logger ${dependencies})
target_link_libraries(logger plansys2_logger)

install(TARGETS
  plansys2_logger logger
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_package()
