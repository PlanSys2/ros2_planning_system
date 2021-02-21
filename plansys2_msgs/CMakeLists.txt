project(plansys2_msgs)

cmake_minimum_required(VERSION 3.5)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ActionExecution.msg"
  "msg/ActionExecutionInfo.msg"
  "msg/ActionPerformerStatus.msg"
  "msg/Knowledge.msg"
  "srv/GetDomain.srv"
  "srv/GetDomainTypes.srv"
  "srv/GetDomainPredicates.srv"
  "srv/GetDomainPredicateDetails.srv"
  "srv/GetDomainFunctions.srv"
  "srv/GetDomainFunctionDetails.srv"
  "srv/GetDomainActions.srv"
  "srv/GetDomainActionDetails.srv"
  "srv/GetProblem.srv"
  "srv/GetProblemInstances.srv"
  "srv/GetProblemInstanceDetails.srv"
  "srv/GetProblemPredicates.srv"
  "srv/GetProblemPredicateDetails.srv"
  "srv/GetProblemFunctions.srv"
  "srv/GetProblemFunctionDetails.srv"
  "srv/GetProblemGoal.srv"
  "srv/IsProblemGoalSatisfied.srv"
  "srv/AddProblemGoal.srv"
  "srv/AddProblemInstance.srv"
  "srv/AddProblemPredicate.srv"
  "srv/AddProblemFunction.srv"
  "srv/RemoveProblemGoal.srv"
  "srv/ClearProblemKnowledge.srv"
  "srv/RemoveProblemInstance.srv"
  "srv/RemoveProblemPredicate.srv"
  "srv/RemoveProblemFunction.srv"
  "srv/ExistProblemPredicate.srv"
  "srv/ExistProblemFunction.srv"
  "srv/UpdateProblemFunction.srv"
  "srv/GetPlan.srv"
  "action/ExecutePlan.action"
  DEPENDENCIES builtin_interfaces std_msgs action_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
