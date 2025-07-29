cmake_minimum_required(VERSION 3.5)

project(plansys2_tools)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(qt_gui_cpp REQUIRED)
find_package(rqt_gui_cpp REQUIRED)
find_package(plansys2_problem_expert REQUIRED)

find_package(Qt5 REQUIRED COMPONENTS Widgets)

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_library(plansys2_logger SHARED src/plansys2_logger/LoggerNode.cpp)
target_include_directories(plansys2_logger PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(plansys2_logger
  PUBLIC
  ${plansys2_msgs_TARGETS}
  rclcpp::rclcpp
)

add_executable(logger src/logger.cpp)
target_include_directories(logger PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(logger
  PRIVATE
  plansys2_logger
  rclcpp::rclcpp
)

# RQT Knowledge
set(rqt_plansys2_knowledge_SRCS
  src/rqt_plansys2_knowledge/RQTKnowledge.cpp
  src/rqt_plansys2_knowledge/KnowledgeTree.cpp
)

set(rqt_plansys2_knowledge_HDRS
  include/rqt_plansys2_knowledge/RQTKnowledge.hpp
  include/rqt_plansys2_knowledge/KnowledgeTree.hpp
)

set(rqt_plansys2_knowledge_UIS
  src/rqt_plansys2_knowledge/rqt_plansys2_knowledge.ui
)

qt5_wrap_cpp(rqt_plansys2_knowledge_MOCS ${rqt_plansys2_knowledge_HDRS})
qt5_wrap_ui(rqt_plansys2_knowledge_UIS_H ${rqt_plansys2_knowledge_UIS})

add_library(rqt_plansys2_knowledge SHARED
  ${rqt_plansys2_knowledge_SRCS}
  ${rqt_plansys2_knowledge_MOCS}
  ${rqt_plansys2_knowledge_UIS_H}
)
target_include_directories(rqt_plansys2_knowledge PUBLIC
  ${Qt5Widgets_INCLUDE_DIRS}
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(rqt_plansys2_knowledge
  PUBLIC
  plansys2_problem_expert::plansys2_problem_expert
  ${plansys2_msgs_TARGETS}
  rqt_gui_cpp::rqt_gui_cpp
  rclcpp::rclcpp
  Qt5::Widgets
  PRIVATE
  pluginlib::pluginlib
)

# RQT Performers
set(rqt_plansys2_performers_SRCS
  src/rqt_plansys2_performers/RQTPerformers.cpp
  src/rqt_plansys2_performers/PerformersTree.cpp
)

set(rqt_plansys2_performers_HDRS
  include/rqt_plansys2_performers/RQTPerformers.hpp
  include/rqt_plansys2_performers/PerformersTree.hpp
)

set(rqt_plansys2_performers_UIS
  src/rqt_plansys2_performers/rqt_plansys2_performers.ui
)

qt5_wrap_cpp(rqt_plansys2_performers_MOCS ${rqt_plansys2_performers_HDRS})
qt5_wrap_ui(rqt_plansys2_performers_UIS_H ${rqt_plansys2_performers_UIS})

add_library(rqt_plansys2_performers SHARED
  ${rqt_plansys2_performers_SRCS}
  ${rqt_plansys2_performers_MOCS}
  ${rqt_plansys2_performers_UIS_H}
)
target_include_directories(rqt_plansys2_performers PUBLIC
  ${Qt5Widgets_INCLUDE_DIRS}
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(rqt_plansys2_performers
  PUBLIC
  plansys2_problem_expert::plansys2_problem_expert
  ${plansys2_msgs_TARGETS}
  rqt_gui_cpp::rqt_gui_cpp
  rclcpp::rclcpp
  Qt5::Widgets
  PRIVATE
  pluginlib::pluginlib
)

# RQT Plan
set(rqt_plansys2_plan_SRCS
  src/rqt_plansys2_plan/RQTPlan.cpp
  src/rqt_plansys2_plan/PlanTree.cpp
)

set(rqt_plansys2_plan_HDRS
  include/rqt_plansys2_plan/RQTPlan.hpp
  include/rqt_plansys2_plan/PlanTree.hpp
)

set(rqt_plansys2_plan_UIS
  src/rqt_plansys2_plan/rqt_plansys2_plan.ui
)

qt5_wrap_cpp(rqt_plansys2_plan_MOCS ${rqt_plansys2_plan_HDRS})
qt5_wrap_ui(rqt_plansys2_plan_UIS_H ${rqt_plansys2_plan_UIS})

add_library(rqt_plansys2_plan SHARED
  ${rqt_plansys2_plan_SRCS}
  ${rqt_plansys2_plan_MOCS}
  ${rqt_plansys2_plan_UIS_H}
)
target_include_directories(rqt_plansys2_plan PUBLIC
  ${Qt5Widgets_INCLUDE_DIRS}
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(rqt_plansys2_plan
  PUBLIC
  ${plansys2_msgs_TARGETS}
  rqt_gui_cpp::rqt_gui_cpp
  rclcpp::rclcpp
  Qt5::Widgets
  PRIVATE
  pluginlib::pluginlib
)

install(TARGETS
  plansys2_logger logger
  rqt_plansys2_knowledge
  rqt_plansys2_performers
  rqt_plansys2_plan
  EXPORT rqt_plansys2
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(PROGRAMS
  scripts/rqt_plansys2_knowledge
  scripts/rqt_plansys2_performers
  scripts/rqt_plansys2_plan
  DESTINATION bin
)

install(FILES plugin_knowledge.xml plugin_performers.xml plugin_plan.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
endif()

pluginlib_export_plugin_description_file(rqt_gui "plugin_knowledge.xml")
pluginlib_export_plugin_description_file(rqt_gui "plugin_performers.xml")
pluginlib_export_plugin_description_file(rqt_gui "plugin_plan.xml")
ament_export_libraries(rqt_plansys2)
ament_export_targets(rqt_plansys2)
ament_export_dependencies(
  Qt5
  rclcpp
  plansys2_msgs
  qt_gui_cpp
  rqt_gui_cpp
  plansys2_problem_expert
)

ament_package()
