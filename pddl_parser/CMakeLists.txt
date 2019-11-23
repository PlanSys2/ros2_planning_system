cmake_minimum_required(VERSION 3.5)
project(pddl_parser)

set(CMAKE_BUILD_TYPE DEBUG)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

set(dependencies
    rclcpp
)

include_directories(include)

set(PDDL-PARSER-SOURCES
  src/pddl_parser/GroundFunc.cpp
  src/pddl_parser/When.cpp
  src/pddl_parser/Equals.cpp
  src/pddl_parser/Function.cpp
  src/pddl_parser/Expression.cpp
  src/pddl_parser/ParamCond.cpp
  src/pddl_parser/Action.cpp
  src/pddl_parser/TemporalAction.cpp
  src/pddl_parser/Or.cpp
  src/pddl_parser/Exists.cpp
  src/pddl_parser/Not.cpp
  src/pddl_parser/Derived.cpp
  src/pddl_parser/And.cpp
  src/pddl_parser/Lifted.cpp
  src/pddl_parser/Forall.cpp
  src/pddl_parser/Oneof.cpp
  src/pddl_parser/FunctionModifier.cpp
  src/pddl_parser/Ground.cpp
  src/pddl_parser/TypeGround.cpp
)

add_library(${PROJECT_NAME} SHARED ${PDDL-PARSER-SOURCES})

add_executable(parser src/parser.cpp)
target_link_libraries(parser ${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/
)

install(TARGETS
  ${PROJECT_NAME}
  parser
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

#if(BUILD_TESTING)
#  find_package(ament_lint_auto REQUIRED)
#  ament_lint_auto_find_test_dependencies()
#
#  find_package(ament_cmake_gtest REQUIRED)
#  add_subdirectory(test)
#endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()
