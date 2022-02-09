project(plansys2_tools)
cmake_minimum_required(VERSION 3.5)

find_package(Threads REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(qt_gui_cpp REQUIRED)
find_package(rqt_gui_cpp REQUIRED)
find_package(plansys2_problem_expert REQUIRED)

find_package(Qt5Widgets REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE DEBUG)

set(dependencies
  rclcpp
  plansys2_msgs
  qt_gui_cpp
  rqt_gui_cpp
  plansys2_problem_expert
)

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
include_directories(include)

add_library(plansys2_logger SHARED src/plansys2_logger/LoggerNode.cpp)
ament_target_dependencies(plansys2_logger ${dependencies})

add_executable(logger src/logger.cpp)
ament_target_dependencies(logger ${dependencies})
target_link_libraries(logger plansys2_logger)


## RQT Knowledge

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

set(rqt_plansys2_knowledge_INCLUDE_DIRECTORIES
  include
  ${Qt5Widgets_INCLUDE_DIRS}
)

qt5_wrap_cpp(rqt_plansys2_knowledge_MOCS ${rqt_plansys2_knowledge_HDRS})
qt5_wrap_ui(rqt_plansys2_knowledge_UIS_H ${rqt_plansys2_knowledge_UIS})

include_directories(
  ${rqt_plansys2_knowledge_INCLUDE_DIRECTORIES}
)

add_library(rqt_plansys2_knowledge SHARED
  ${rqt_plansys2_knowledge_SRCS}
  ${rqt_plansys2_knowledge_MOCS}
  ${rqt_plansys2_knowledge_UIS_H}
)

target_link_libraries(
  rqt_plansys2_knowledge
  Qt5::Widgets
)

ament_target_dependencies(rqt_plansys2_knowledge ${dependencies})



install(TARGETS
  plansys2_logger logger
  rqt_plansys2_knowledge
  EXPORT rqt_plansys2_knowledge
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS scripts/rqt_plansys2_knowledge
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
endif()

pluginlib_export_plugin_description_file(rqt_gui "plugin.xml")
ament_export_libraries(rqt_plansys2_knowledge)

ament_package()
