cmake_minimum_required(VERSION 3.5)
project(plansys2_msgs)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetDomain.srv"
  "srv/GetDomainTypes.srv"
  "srv/GetDomainPredicates.srv"
  "srv/GetDomainPredicateDetails.srv"
  "srv/GetDomainActions.srv"
  "srv/GetDomainActionDetails.srv"
  DEPENDENCIES builtin_interfaces std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
