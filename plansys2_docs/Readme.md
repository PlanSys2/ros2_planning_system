# ROS2 Planning System Design document

## Requirements

* The planning system must to support PDDL 2.1, including:
  * Durative actions
  * Metrics
* The planning system can be queried for domain details using services:
  * Types
  * Predicates and their details (id, arguments and their types)
  * Actions details (preconditions predicates, arguments, efects, ...)
  * To use several domain files (mix domains and warnings when collisions)
  * Services:
    * `/planning_system/domain_expert/get_list_predicates`
    * `/planning_system/domain_expert/get_predicate_details`
    * `/planning_system/domain_expert/get_list_actions`
    * `/planning_system/domain_expert/get_action_details`
    * `/planning_system/domain_expert/get_list_durative_actions`
    * `/planning_system/domain_expert/get_durative_action_details`
* The planning system has a knowledge base, with this functionality:
  * It contains the current predicates, instances and the current goal
  * Services:
    * `/planning_system/knowledge_base/get_list_predicates`
    * `/planning_system/knowledge_base/get_predicate_details`
    * `/planning_system/knowledge_base/add_predicate`
    * `/planning_system/knowledge_base/remove_predicate`
    * `/planning_system/knowledge_base/get_list_instances`
    * `/planning_system/knowledge_base/get_instances_details`
    * `/planning_system/knowledge_base/add_instance`
    * `/planning_system/knowledge_base/remove_instances`
    * `/planning_system/knowledge_base/get_list_goals`
    * `/planning_system/knowledge_base/get_goal_details`
    * `/planning_system/knowledge_base/add_goal`
    * `/planning_system/knowledge_base/remove_goal`
* The planning system has a planner, with this functionality:
    * Create a plan using the current domain and problem using an action
        * Action with feedback of how the actions is going (number of action in sequence, current status of the action)
    * Deliver actions in sequence to the action performers
    * Check in durative actions the conditions at runtime. Cancel execution and replanning. If no plan possible, return fail
* Action performers
    * Nodes that handle the activation of each action
    * Interface by actions
* Clients to avoid complexity of using services.




  

## Use cases

## Structure

