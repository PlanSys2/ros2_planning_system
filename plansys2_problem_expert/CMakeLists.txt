cmake_minimum_required(VERSION 3.5)
project(plansys2_problem_expert)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE DEBUG)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(pddl_parser REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_domain_expert REQUIRED)

set(dependencies
    rclcpp
    rclcpp_lifecycle
    pddl_parser
    ament_index_cpp
    plansys2_msgs
    plansys2_domain_expert
)

include_directories(include)

set(PROBLEM_EXPERT_SOURCES
  src/plansys2_problem_expert/ProblemExpert.cpp
)

add_library(${PROJECT_NAME} SHARED ${PROBLEM_EXPERT_SOURCES})
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

#add_executable(problem_expert_node
#  src/problem_expert_node.cpp
#  src/plansys2_problem_expert/DomainExpertNode.cpp)
#ament_target_dependencies(problem_expert_node ${dependencies})
#target_link_libraries(problem_expert_node ${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}
#  problem_expert_node
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