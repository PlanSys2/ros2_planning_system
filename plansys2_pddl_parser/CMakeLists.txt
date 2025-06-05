cmake_minimum_required(VERSION 3.5)

project(plansys2_pddl_parser)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(dependencies
    rclcpp::rclcpp
    ${plansys2_msgs_TARGETS}
    ${std_msgs_TARGETS}
)

set(package_dependencies
  rclcpp
  plansys2_msgs
  std_msgs
)

include_directories(include)

set(PDDL-PARSER-SOURCES
  src/plansys2_pddl_parser/GroundFunc.cpp
  src/plansys2_pddl_parser/When.cpp
  src/plansys2_pddl_parser/Function.cpp
  src/plansys2_pddl_parser/Expression.cpp
  src/plansys2_pddl_parser/ParamCond.cpp
  src/plansys2_pddl_parser/Action.cpp
  src/plansys2_pddl_parser/TemporalAction.cpp
  src/plansys2_pddl_parser/Or.cpp
  src/plansys2_pddl_parser/Exists.cpp
  src/plansys2_pddl_parser/Not.cpp
  src/plansys2_pddl_parser/Derived.cpp
  src/plansys2_pddl_parser/And.cpp
  src/plansys2_pddl_parser/Lifted.cpp
  src/plansys2_pddl_parser/Forall.cpp
  src/plansys2_pddl_parser/Oneof.cpp
  src/plansys2_pddl_parser/FunctionModifier.cpp
  src/plansys2_pddl_parser/Ground.cpp
  src/plansys2_pddl_parser/Utils.cpp
  src/plansys2_pddl_parser/TypeGround.cpp
  src/plansys2_pddl_parser/Imply.cpp
)

add_library(${PROJECT_NAME} SHARED ${PDDL-PARSER-SOURCES})
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${PROJECT_NAME} PUBLIC ${dependencies})

add_executable(parser src/parser.cpp)
target_link_libraries(parser PUBLIC ${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  parser
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_index_cpp REQUIRED)
  set(dependencies ${dependencies} ament_index_cpp::ament_index_cpp)
  add_subdirectory(test)
endif()

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(${package_dependencies})

ament_package()
