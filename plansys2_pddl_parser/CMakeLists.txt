cmake_minimum_required(VERSION 3.5)
project(plansys2_pddl_parser)

set(CMAKE_BUILD_TYPE DEBUG)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

set(dependencies
    rclcpp
)

include_directories(include)

set(PDDL-PARSER-SOURCES
  src/plansys2_pddl_parser/GroundFunc.cpp
  src/plansys2_pddl_parser/When.cpp
  src/plansys2_pddl_parser/Equals.cpp
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
  src/plansys2_pddl_parser/TypeGround.cpp
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
