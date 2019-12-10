# Domain Expert

The Domain Expert module is responsible for maintaining the PDDL domain. 

The main class is [`plansys2::DomainExpertNode`](include/include/plansys2_domain_expert/DomainExpertNode.hpp), which is instantiated from [`domain_expert_node.cpp`](src/domain_expert_node.cpp). `plansys2::DomainExpertNode` is a also `rclcpp_lifecycle::LifecycleNode`, but currently the functionality is in the active phase.

The class responsible for maintaining this domain is [`plansys2::ProblemExpert`](include/include/plansys2_problem_expert/ProblemExpert.hpp), which is independent of ROS2.

The Problem Expert is dynamic and volatile, accessing its functionality through ROS2 services. To facilitate the task of the application developer, an [`plansys2::ProblemExpertClient`](include/include/plansys2_problem_expert/ProblemExpertClient.hpp) class has been implemented that hides the complexity of handling ROS2 messages and services. Its API is similar to that of [`plansys2::ProblemExpert`](include/include/plansys2_problem_expert/ProblemExpert.hpp), since both have to implement the [`plansys2::ProblemExpertInterface`](include/include/plansys2_problem_expert/ProblemExpertInterface.hpp) interface.

The Problem Expert instantiates a [`plansys2::DomainExpertClient`](include/include/plansys2_domain_expert/DomainExpertClient.hpp), and every update query is verified against domain to check if it is valid.

Every update in the Problem, is notified publishing a `std_msgs::msg::Empty` in `/problem_expert/update_notify`. It helps other modules and applications to be aware of updates, being not necessary to do polling to check it.

## Services:

- `/problem_expert/add_problem_goal` [[`plansys2_msgs::srv::AddProblemGoal`](../plansys2_msgs/srv/AddProblemGoal.srv)]
- `/problem_expert/add_problem_instance` [[`plansys2_msgs::srv::AddProblemInstance`](../plansys2_msgs/srv/AddProblemInstance.srv)]
- `/problem_expert/add_problem_predicate` [[`plansys2_msgs::srv::AddProblemPredicate`](../plansys2_msgs/srv/AddProblemPredicate.srv)]
- `/problem_expert/get_problem_goal` [[`plansys2_msgs::srv::GetProblemGoal`](../plansys2_msgs/srv/GetProblemGoal.srv)]
- `/problem_expert/get_problem_instance_details` [[`plansys2_msgs::srv::GetProblemInstanceDetails`](../plansys2_msgs/srv/GetProblemInstanceDetails.srv)]
- `/problem_expert/get_problem_instances` [[`plansys2_msgs::srv::GetProblemInstances`](../plansys2_msgs/srv/GetProblemInstances.srv)]
- `/problem_expert/get_problem_predicate_details` [[`plansys2_msgs::srv::GetProblemPredicateDetails`](../plansys2_msgs/srv/GetProblemPredicateDetails.srv)]
- `/problem_expert/get_problem` [[`plansys2_msgs::srv::GetProblem`](../plansys2_msgs/srv/GetProblem.srv)]
- `/problem_expert/remove_problem_goal` [[`plansys2_msgs::srv::RemoveProblemGoal`](../plansys2_msgs/srv/RemoveProblemGoal.srv)]
- `/problem_expert/remove_problem_instance` [[`plansys2_msgs::srv::RemoveProblemInstance`](../plansys2_msgs/srv/RemoveProblemInstance.srv)]
- `/problem_expert/remove_problem_predicate` [[`plansys2_msgs::srv::RemoveProblemPredicate](../plansys2_msgs/srv/RemoveProblemPredicate.srv)]
- `/problem_expert/exist_problem_predicate` [[`plansys2_msgs::srv::ExistProblemPredicate`](../plansys2_msgs/srv/ExistProblemPredicate.srv)]

## Published topics

- `/problem_expert/update_notify` [`std_msgs::msg::Empty`]
