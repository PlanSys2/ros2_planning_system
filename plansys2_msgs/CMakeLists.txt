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
  "srv/GetProblem.srv"
  "srv/GetProblemInstances.srv"
  "srv/GetProblemInstanceDetails.srv"
  "srv/GetProblemPredicates.srv"
  "srv/GetProblemPredicateDetails.srv"
  "srv/GetProblemGoal.srv"
  "srv/AddProblemGoal.srv"
  "srv/AddProblemInstance.srv"
  "srv/AddProblemPredicate.srv"
  "srv/RemoveProblemGoal.srv"
  "srv/RemoveProblemInstance.srv"
  "srv/RemoveProblemPredicate.srv"
  "srv/ExistProblemPredicate.srv"
  "srv/GetPlan.srv"
  "action/ExecutePlan.action"
  "action/ExecuteAction.action" 
  DEPENDENCIES builtin_interfaces std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
