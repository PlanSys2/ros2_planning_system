cmake_minimum_required(VERSION 3.16)

project(plansys2_msgs)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Action.msg"
  "msg/ActionExecution.msg"
  "msg/ActionExecutionInfo.msg"
  "msg/ActionPerformerStatus.msg"
  "msg/Derived.msg"
  "msg/DurativeAction.msg"
  "msg/ExecutionEvent.msg"
  "msg/Knowledge.msg"
  "msg/Metric.msg"
  "msg/Node.msg"
  "msg/Param.msg"
  "msg/Plan.msg"
  "msg/PlanArray.msg"
  "msg/PlanItem.msg"
  "msg/Problem.msg"
  "msg/State.msg"
  "msg/Tree.msg"
  "srv/AddProblem.srv"
  "srv/SetContinuousEffect.srv"
  "srv/AddProblemGoal.srv"
  "srv/AffectNode.srv"
  "srv/AffectParam.srv"
  "srv/ExistNode.srv"
  "srv/GetDomain.srv"
  "srv/GetDomainActions.srv"
  "srv/GetDomainActionDetails.srv"
  "srv/GetDomainDerivedPredicateDetails.srv"
  "srv/GetDomainDurativeActionDetails.srv"
  "srv/GetDomainName.srv"
  "srv/GetDomainTypes.srv"
  "srv/GetDomainConstants.srv"
  "srv/GetNodeDetails.srv"
  "srv/GetPlan.srv"
  "srv/GetPlanArray.srv"
  "srv/GetOrderedSubGoals.srv"
  "srv/GetProblem.srv"
  "srv/GetProblemGoal.srv"
  "srv/GetProblemInstances.srv"
  "srv/GetProblemInstanceDetails.srv"
  "srv/GetProblemMetric.srv"
  "srv/GetStates.srv"
  "srv/IsProblemGoalSatisfied.srv"
  "srv/RemoveProblemGoal.srv"
  "srv/RemoveProblemMetric.srv"
  "srv/SetProblemMetric.srv"
  "srv/ClearProblemKnowledge.srv"
  "srv/ValidateDomain.srv"
  "srv/ChangeDomain.srv"
  "action/ExecutePlan.action"
  DEPENDENCIES builtin_interfaces std_msgs action_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
