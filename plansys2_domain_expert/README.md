# Domain Expert

The Domain Expert module is responsible for maintaining the PDDL domain. 

The main class is [`plansys2::DomainExpertNode`](include/plansys2_domain_expert/DomainExpertNode.hpp), which is instantiated from [`DomainExpertNode.cpp`](src/DomainExpertNode.cpp). `plansys2::DomainExpertNode` is a `rclcpp_lifecycle::LifecycleNode` and in its configuration phase reads the `model_file` parameter, which contains the .pddl file from which to read the model.

The class responsible for maintaining this domain is [`plansys2::DomainExpert`](include/plansys2_domain_expert/DomainExpert.hpp), which is independent of ROS2.

The Domain Expert does not change while active, accessing its functionality through ROS2 services. To facilitate the task of the application developer, an [`plansys2::DomainExpertClient`](include/plansys2_domain_expert/DomainExpertClient.hpp) class has been implemented that hides the complexity of handling ROS2 messages and services. Its API is similar to that of [`plansys2::DomainExpert`](include/plansys2_domain_expert/DomainExpert.hpp), since both have to implement the [`plansys2::DomainExpertInterface`](include/plansys2_domain_expert/DomainExpertInterface.hpp) interface.

## Services:

- `/domain_expert/get_domain` [[`plansys2_msgs::srv::GetDomain`](../plansys2_msgs/srv/GetDomain.srv)]
- `/domain_expert/get_domain_action_details` [[`plansys2_msgs::srv::GetDomainActionDetails`](../plansys2_msgs/srv/GetDomainActionDetails.srv)]
- `/domain_expert/get_domain_actions` [[`plansys2_msgs::srv::GetDomainActions`](../plansys2_msgs/srv/GetDomainActions.srv)]
- `/domain_expert/get_domain_durative_action_details` [[`plansys2_msgs::srv::GetDomainDurativeActionDetails`](../plansys2_msgs/srv/GetDomainDurativeActionDetails.srv)]
- `/domain_expert/get_domain_durative_actions` [[`plansys2_msgs::srv::GetDomainActions`](../plansys2_msgs/srv/GetDomainActions.srv)]
- `/domain_expert/get_domain_function_details` [[`plansys2_msgs::srv::GetNodeDetails`](../plansys2_msgs/srv/GetNodeDetails.srv)]
- `/domain_expert/get_domain_functions` [[`plansys2_msgs::srv::GetStates`](../plansys2_msgs/srv/GetStates.srv)]
- `/domain_expert/get_domain_predicate_details` [[`plansys2_msgs::srv::GetNodeDetails`](../plansys2_msgs/srv/GetNode.srv)]
- `/domain_expert/get_domain_predicates` [[`plansys2_msgs::srv::GetStates`](../plansys2_msgs/srv/GetStates.srv)]
- `/domain_expert/get_domain_types` [[`plansys2_msgs::srv::GetDomainTypes`](../plansys2_msgs/srv/GetDomainTypes.srv)]
- `/domain_expert/get_domain_constants` [[`plansys2_msgs::srv::GetDomainConstants`](../plansys2_msgs/srv/GetDomainConstants.srv)]
