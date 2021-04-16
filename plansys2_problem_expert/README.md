# Problem Expert

The Problem Expert module is responsible for maintaining the instances, predicates and goals of the PDDL problem.

The main class is [`plansys2::ProblemExpertNode`](include/plansys2_problem_expert/ProblemExpertNode.hpp), which is instantiated from [`ProblemExpertNode.cpp`](src/ProblemExpertNode.cpp). `plansys2::ProblemExpertNode` is a `rclcpp_lifecycle::LifecycleNode`, but currently the functionality is in the active phase.

The class responsible for maintaining this problem instance is [`plansys2::ProblemExpert`](include/plansys2_problem_expert/ProblemExpert.hpp), which is independent of ROS2.

The Problem Expert is dynamic and volatile, accessing its functionality through ROS2 services. To facilitate the task of the application developer, an [`plansys2::ProblemExpertClient`](include/plansys2_problem_expert/ProblemExpertClient.hpp) class has been implemented that hides the complexity of handling ROS2 messages and services. Its API is similar to that of [`plansys2::ProblemExpert`](include/plansys2_problem_expert/ProblemExpert.hpp), since both have to implement the [`plansys2::ProblemExpertInterface`](include/include/plansys2_problem_expert/ProblemExpertInterface.hpp) interface.

The Problem Expert instantiates a [`plansys2::DomainExpertClient`](include/include/plansys2_domain_expert/DomainExpertClient.hpp), and every update query is verified against domain to check if it is valid.

Every update in the Problem, is notified publishing a `std_msgs::msg::Empty` in `/problem_expert/update_notify`. It helps other modules and applications to be aware of updates, being not necessary to do polling to check it.

## Services

- `/problem_expert/add_problem_function` [[`plansys2_msgs::srv::AffectNode`](../plansys2_msgs/srv/AffectNode.srv)]
- `/problem_expert/add_problem_goal` [[`plansys2_msgs::srv::AddProblemGoal`](../plansys2_msgs/srv/AddProblemGoal.srv)]
- `/problem_expert/add_problem_instance` [[`plansys2_msgs::srv::AffectParam`](../plansys2_msgs/srv/AffectParam.srv)]
- `/problem_expert/add_problem_predicate` [[`plansys2_msgs::srv::AffectNode`](../plansys2_msgs/srv/AffectNode.srv)]
- `/problem_expert/clear_problem_knowledge` [[`plansys2_msgs::srv::ClearProblemKnowledge`](../plansys2_msgs/srv/ClearProblemKnowledge.srv)]
- `/problem_expert/exist_problem_function` [[`plansys2_msgs::srv::ExistNode`](../plansys2_msgs/srv/ExistNode.srv)]
- `/problem_expert/exist_problem_predicate` [[`plansys2_msgs::srv::ExistNode`](../plansys2_msgs/srv/ExistNode.srv)]
- `/problem_expert/get_problem` [[`plansys2_msgs::srv::GetProblem`](../plansys2_msgs/srv/GetProblem.srv)]
- `/problem_expert/get_problem_function` [[`plansys2_msgs::srv::GetNodeDetails`](../plansys2_msgs/srv/GetNodeDetails.srv)]
- `/problem_expert/get_problem_functions` [[`plansys2_msgs::srv::GetStates`](../plansys2_msgs/srv/GetStates.srv)]
- `/problem_expert/get_problem_goal` [[`plansys2_msgs::srv::GetProblemGoal`](../plansys2_msgs/srv/GetProblemGoal.srv)]
- `/problem_expert/get_problem_instance` [[`plansys2_msgs::srv::GetProblemInstanceDetails`](../plansys2_msgs/srv/GetProblemInstanceDetails.srv)]
- `/problem_expert/get_problem_instances` [[`plansys2_msgs::srv::GetProblemInstances`](../plansys2_msgs/srv/GetProblemInstances.srv)]
- `/problem_expert/get_problem_predicate` [[`plansys2_msgs::srv::GetNodeDetails`](../plansys2_msgs/srv/GetNodeDetails.srv)]
- `/problem_expert/get_problem_predicates` [[`plansys2_msgs::srv::GetStates`](../plansys2_msgs/srv/GetStates.srv)]
- `/problem_expert/is_problem_goal_satisfied` [[`plansys2_msgs::srv::IsProblemGoalSatisfied`](../plansys2_msgs/srv/IsProblemGoalSatisfied.srv)]
- `/problem_expert/remove_problem_function` [[`plansys2_msgs::srv::AffectNode`](../plansys2_msgs/srv/AffectNode.srv)]
- `/problem_expert/remove_problem_goal` [[`plansys2_msgs::srv::RemoveProblemGoal`](../plansys2_msgs/srv/RemoveProblemGoal.srv)]
- `/problem_expert/remove_problem_instance` [[`plansys2_msgs::srv::AffectParam`](../plansys2_msgs/srv/AffectParam.srv)]
- `/problem_expert/remove_problem_predicate` [[`plansys2_msgs::srv::AffectNode`](../plansys2_msgs/srv/AffectNode.srv)]
- `/problem_expert/update_problem_function` [[`plansys2_msgs::srv::AffectNode`](../plansys2_msgs/srv/AffectNode.srv)]

## Published topics

- `/problem_expert/update_notify` [`std_msgs::msg::Empty`]
