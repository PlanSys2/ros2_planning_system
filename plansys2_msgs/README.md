# PlanSys2 Messages

## Representing PDDL Expressions as Trees

PDDL expressions are conveniently represented as trees.
To see this, consider the following PDDL expression.

    (and (robot_at r2d2 millennium_falcon)(not (robot_at r2d2 death_star)))

The root node of the tree corresponds to the outer parentheses encompassing the "and" statement.
The "and" statement then has two children.
The first child corresponds to the predicate `(robot_at r2d2 millennium_falcon)` and the second
child corresponds to the "not" expression `(not (robot_at r2d2 death_star))`.
Similarly, the "not" expression has one child corresponding to the predicate `(robot_at r2d2 death_star)`.

A tree node is often implemented as a simple class object of the form

    class Node {
      public String name;
      public Node[] children;
    }

Thus, the node contains the data relevant to the node as well as the links to its children.
A tree is then stored as a list of nodes.

## Representing PDDL Expressions with ROS Messages

PlanSys2 requires that the PDDL construct trees be passed back and forth between the various nodes of the system.
To define a PDDL tree in ROS, we use two custom messages, `plansys2_msgs/Tree` and `plansys2_msgs/Node`.
The contents of `plansys2_msgs/Tree` is simply an array of type `plansys2_msgs/Node`.

    plansys2_msgs/Node[] nodes

The contents of `plansys2_msgs/Node` contains the data relevant to the node as well as links to its children.

    uint8 node_type
    uint8 expression_type
    uint8 modifier_type
    
    uint32 node_id
    uint32[] children
    
    string name
    plansys2_msgs/Param[] parameters
    float64 value
    bool negate

The `node_type` defines the type of PDDL construct that the node represents.
The following values are currently supported by PlanSys2.

    uint8 AND = 1
    uint8 OR = 2
    uint8 NOT = 3
    uint8 ACTION = 4
    uint8 PREDICATE = 5
    uint8 FUNCTION = 6
    uint8 EXPRESSION = 7
    uint8 FUNCTION_MODIFIER = 8
    uint8 NUMBER = 9

When the `node_type` is `EXPRESSION`, the `expr_type` field is used to define the type of expression represented.
The following expressions types are currently supported by PlanSys2.

    uint8 COMP_GE = 10
    uint8 COMP_GT = 11
    uint8 COMP_LE = 12
    uint8 COMP_LT = 13
    uint8 ARITH_MULT = 14
    uint8 ARITH_DIV = 15
    uint8 ARITH_ADD = 16
    uint8 ARITH_SUB = 17

When the `node_type` is `FUNCTION_MODIFIER` the `modifier_type` field is used to define the type of function modifier represented.
The following function modifier types are currently supported by PlanSys2.

    uint8 ASSIGN = 18
    uint8 INCREASE = 19
    uint8 DECREASE = 20
    uint8 SCALE_UP = 21
    uint8 SCALE_DOWN = 22

The `node_id` corresponds to the node's location in the nodes list and takes on a value in the range [0, len(nodes)-1].

The `children` list specifies the locations, or node id's, of the node's children.

The `name` string and `parameters` list are used by predicate and function nodes, which are defined
by a name and a list of parameters.

The `value` field is used by PDDL functions and stores a numeric value.

Finally, the `negate` field tracks whether the inverse truth value should be used.
For example, if a `NOT` node is the parent of a `PREDICATE` node, then the `PREDICATE` node will
have a `negate` value opposite that of the `NOT` node's value.

## Actions

[`plansys2_msgs::action::ExecutePlan`](../plansys2_msgs/action/ExecutePlan.action)

* Used to start and continuously monitor the execution of a plan.

## Messages

[`plansys2_msgs::msg::Action`](../plansys2_msgs/msg/Action.msg)

* Used to specify a PDDL action, with preconditions and effects defined by [`plansys2_msgs::msg::Tree`](../plansys2_msgs/msg/Tree.msg).

[`plansys2_msgs::msg::ActionExecution`](../plansys2_msgs/msg/ActionExecution.msg)

* Used to establish a communication protocol between the action executors (implemented as behavior
trees) and the action executor clients (user provided lifecyle nodes that perform the actions).

[`plansys2_msgs::msg::ActionExecutionInfo`](../plansys2_msgs/msg/ActionExecutionInfo.msg)

* Used to provide feedback regarding the execution status of each action execution client.

[`plansys2_msgs::msg::ActionPerformerStatus`](../plansys2_msgs/msg/ActionPerformerStatus.msg)

* Used to provide status feedback from an action executor client to the executor.

[`plansys2_msgs::msg::DurativeAction`](../plansys2_msgs/msg/DurativeAction.msg)

* Used to specify a PDDL durative action, with requirements and effects defined by [`plansys2_msgs::msg::Tree`](../plansys2_msgs/msg/Tree.msg).

[`plansys2_msgs::msg::Knowledge`](../plansys2_msgs/msg/Knowledge.msg)

* This message is not used internally by the system, but provides feedback to the user when a
knowledge item is added, removed, or modified.

[`plansys2_msgs::msg::Node`](../plansys2_msgs/msg/Node.msg)

* The base node message used for creating PDDL construct trees.
See discussion above for details.

[`plansys2_msgs::msg::Param`](../plansys2_msgs/msg/Param.msg)

* Used to define PDDL parameters and instances.

[`plansys2_msgs::msg::Tree`](../plansys2_msgs/msg/Tree.msg)

* Consists of a list of [`plansys2_msgs::msg::Node`](../plansys2_msgs/msg/Node.msg) and defines a PDDL construct tree.
See discussion above for details.

## Services

[`plansys2_msgs::srv::AddProblemGoal`](../plansys2_msgs/srv/AddProblemGoal.srv)

* Used to add a goal represented as a [`plansys2_msgs::msg::Tree`](../plansys2_msgs/msg/Tree.msg) to the problem expert.

[`plansys2_msgs::srv::AffectNode`](../plansys2_msgs/srv/AffectNode.srv)

* Used to add, remove, or modify a predicate or a function represented as a [`plansys2_msgs::msg::Node`](../plansys2_msgs/msg/Node.msg).

[`plansys2_msgs::srv::AffectParam`](../plansys2_msgs/srv/AffectParam.srv)

* Used to add or remove a problem instance represented as [`plansys2_msgs::msg::Param`](../plansys2_msgs/msg/Param.msg)

[`plansys2_msgs::srv::ClearProblemKnowledge`](../plansys2_msgs/srv/ClearProblemKnowledge.srv)

* Used to clear the problem instances, predicates, and functions.

[`plansys2_msgs::srv::ExistNode`](../plansys2_msgs/srv/ExistNode.srv)

* Used to check for the existence of a predicate or function represented as a [`plansys2_msgs::msg::Node`](../plansys2_msgs/msg/Node.msg).

[`plansys2_msgs::srv::GetDomain`](../plansys2_msgs/srv/GetDomain.srv)

* Returns the domain as a string.

[`plansys2_msgs::srv::GetDomainActionDetails`](../plansys2_msgs/srv/GetDomainActionDetails.srv)

* Given an action name and an optional list of parameters, returns an action ([`plansys2_msgs::msg::Action`](../plansys2_msgs/msg/Action.msg)).
If the optional parameter list is not empty it will be used to define the returned action parameters.
Otherwise, auto-generated values will be used.
The optional parameter list can be used to retrieve an instantiated plan action.

[`plansys2_msgs::srv::GetDomainActions`](../plansys2_msgs/srv/GetDomainActions.srv)

* Returns a list of the domain action names.

[`plansys2_msgs::srv::GetDomainDurativeActionDetails`](../plansys2_msgs/srv/GetDomainDurativeActionDetails.srv)

* Given a durative action name and an optional list of parameters, returns a durative action ([`plansys2_msgs::msg::Action`](../plansys2_msgs/msg/Action.msg)).
If the optional parameter list is not empty it will be used to define the returned durative action parameters.
Otherwise, auto-generated values will be used.
The optional parameter list can be used to retrieve an instantiated plan action.

[`plansys2_msgs::srv::GetDomainName`](../plansys2_msgs/srv/GetDomainName.srv)

* Returns the domain name.

[`plansys2_msgs::srv::GetDomainTypes`](../plansys2_msgs/srv/GetDomainTypes.srv)

* Returns the domain types.

[`plansys2_msgs::srv::GetDomainConstants`](../plansys2_msgs/srv/GetDomainConstants.srv)

* Returns the domain constants of a type.

[`plansys2_msgs::srv::GetNodeDetails`](../plansys2_msgs/srv/GetNodeDetails.srv)

* Returns a predicate or function node represented as a ([`plansys2_msgs::msg::Node`](../plansys2_msgs/msg/Node.msg)).
When used with the domain expert client, only the name of the node needs to be provided.
When used with the problem expert client, the full construct string must be provided.

[`plansys2_msgs::srv::GetOrderedSubGoals`](../plansys2_msgs/srv/GetOrderedSubGoals.srv)

* Returns an ordered list of sub-goals where each sub-goal is represented as a [`plansys2_msgs::msg::Tree`](../plansys2_msgs/msg/Tree.msg).
The sub-goals are ordered according their start times as specified in the plan.

[`plansys2_msgs::srv::GetPlan`](../plansys2_msgs/srv/GetPlan.srv)

* Returns a generated plan as a list of times, actions (represented as strings), and durations.

[`plansys2_msgs::srv::GetProblem`](../plansys2_msgs/srv/GetProblem.srv)

* Returns the problem as a string.

[`plansys2_msgs::srv::GetProblemGoal`](../plansys2_msgs/srv/GetProblemGoal.srv)

* Returns the entire problem goal represented as a [`plansys2_msgs::msg::Tree`](../plansys2_msgs/msg/Tree.msg).

[`plansys2_msgs::srv::GetProblemInstanceDetails`](../plansys2_msgs/srv/GetProblemInstanceDetails.srv)

* Given a parameter name, returns an instance represented as a ([`plansys2_msgs::msg::Param`](../plansys2_msgs/msg/Param.msg)).

[`plansys2_msgs::srv::GetProblemInstances`](../plansys2_msgs/srv/GetProblemInstances.srv)

* Returns a list of problem instances where each instance is represented as a [`plansys2_msgs::msg::Param`](../plansys2_msgs/msg/Param.msg).

[`plansys2_msgs::srv::GetStates`](../plansys2_msgs/srv/GetStates.srv)

* Used to return a list of predicates or functions where each predicate or function is defined as a [`plansys2_msgs::msg::Node`](../plansys2_msgs/msg/Node.msg).

[`plansys2_msgs::srv::IsProblemGoalSatisfied`](../plansys2_msgs/srv/IsProblemGoalSatisfied.srv)

* Specifies if the given goal, represented as a [`plansys2_msgs::msg::Tree`](../plansys2_msgs/msg/Tree.msg), has been satisfied.

[`plansys2_msgs::srv::RemoveProblemGoal`](../plansys2_msgs/srv/RemoveProblemGoal.srv)

* Removes the problem goal, including all sub-goals.
