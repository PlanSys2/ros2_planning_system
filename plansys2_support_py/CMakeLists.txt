cmake_minimum_required(VERSION 3.5)

project(plansys2_support_py)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(plansys2_msgs REQUIRED)

find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

ament_python_install_package(${PROJECT_NAME})

ament_package()