^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package plansys2_executor
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.0 (2021-07-04)
------------------
* Fix default param
* Solve statically parameter error
* Fix compile issues for galactic
* Compile for ROS2 Galactic
* Contributors: Francisco Martín Rico, Jonatan Olofsson, bjnjo

1.0.10 (2021-07-03)
-------------------
* Minor update
* Fix tests
* Fix rate conversions
* Add rate parameter
* Remove node parameter in client constructors
* ros2-plan-msg: Passing plan to executor to add further separation between plan creation and plan execution.
* Suggestion for `#118 <https://github.com/IntelligentRoboticsLabs/ros2_planning_system/issues/118>`_
* pddl-tree-messages: Performing some minor cleanup.
* pddl-tree-messages: Using explicit specifier for single parameter constructors in plansys2_core/Types.hpp.
* pddl-tree-messages: Replacing user access function calls with shorter versions where possible.
* pddl-tree-messages: Adding predicate user access functions to problem expert client.
* pddl-tree-messages: Adding predicate user access functions to problem expert client.
* pddl-tree-messages: Applying ament uncrustify.
* pddl-tree-messages: Updating addInstance and removeInstance calls to use helper classes.
* pddl-tree-messages: Reverting change to log statement severity.
* pddl-tree-messages: Removing a couple unnecessary includes.
* pddl-tree-messages: Merging master and resolving conflicts.
* action-timeout-clean: Renaming test behavior tree.
* Using custom behavior tree to enable action timeouts.
* pddl-tree-messages: Merging upstream master and resolving conflicts.
* pddl-tree-messages: Using ROS messages to define the PDDL construct trees.
* Adding action execution status (as color changes) to the plan dotgraph.
  Adding legend to plan dotgraph, adding node params for dotgraph legend and printing plan graph to terminal.
* Fix tests and linting
* linting
* Configurable action BT
* Configurable BT Action
* Reducing log message severity because lack of a plan isn't necessarily an error.
* Reduce debug output
* Plansys2_tests package
* Adding unit test for getOrderedSubGoals.
* Add GetOrderedSubGoals service to Executor, allowing executor clients to get the order in which sub-goals will be completed by the current plan.
* Update version
* Contributors: Alexander Xydes, Francisco Martín Rico, Greg Kogut, Josh Zapf

1.0.9 (2021-03-15)
------------------
* Disable boost in tests
* Contributors: Francisco Martín Rico
1.0.8 (2021-03-12)
------------------
* Change default ZMQ ports
* Removing whitespace.
* Simplified logic.
* Update API for FutureReturnCode
* Handling edge cases of action failure and preventing overriding completion percentage on action completion.
* Publishing generated plan as a dotgraph on a string topic.
* Fix BT creation; parallel deps actions
* Fixing feedback control
* Action execution refactoring
* Moving Utils file to plansys2_problem_expert since all the functions are checking information in the problem or modifying the problem.
* utils-bug: Fixing bug in OR case of evaluate function in plansys2_executor/Utils.cpp.
* Making zmq error message more generic to reflect that there are multiple possible reasons for a BT::LogicError to be thrown.
* Add support to plansys2_executor/ExecutorNode for visualizing the behavior trees in Groot.
* Moving publisher on_activate call to the ExecutorNode::on_activate callback.
* Add support for numeric conditions and effects.
* Monitorization info
* Remove an unreshable return
* Adding actor checker in terminal
* Improving BTActions
* Fix repeated nodes
* New Graph creation Algorithm
* Debugging
* Added negative predicates support
* Namespaced action_hub
* Contributors: Alexander Xydes, Fabrice Larribe, Francisco Martin Rico, Greg Kogut, Josh Zapf

1.0.7 (2021-01-04)
------------------
* Making explicit dependencies
* Contributors: Francisco Martín Rico
1.0.6 (2020-12-29)
------------------
* Disable boost functions
* Contributors: Francisco Martín Rico
1.0.5 (2020-12-28)
------------------
* Migration to c++17
* Contributors: Francisco Martín Rico

1.0.4 (2020-12-24)
------------------

1.0.3 (2020-12-23)
------------------

1.0.2 (2020-12-23)
------------------
* Plan solvers as plugins
* Contributors: Fabrice Larribe, Francisco Martin Rico, f269858

1.0.1 (2020-07-19)
------------------

1.0.0 (2020-07-19)
------------------
* Foxy initial version
* Boost:optional
* Support for BT actions
* Contributors: Francisco Martin Rico

0.0.8 (2020-07-18)
------------------
* Add BT support
* Contributors: Francisco Martin Rico

0.0.7 (2020-03-26)
------------------
* ActionExecutorClient is cascade_lifecycle
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Contributors: Francisco Martin Rico

0.0.6 (2020-03-23)
------------------
* Run in separate namespaces. Monolothic node
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Contributors: Francisco Martin Rico

0.0.5 (2020-01-12)
------------------

0.0.4 (2020-01-09)
------------------
* Adding missing action dependencies
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Contributors: Francisco Martín Rico
0.0.3 (2020-01-09)
------------------
* Add popf dependency
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Contributors: Francisco Martín Rico
0.0.2 (2020-01-08)
------------------
* Merge pull request `#16 <https://github.com/IntelligentRoboticsLabs/ros2_planning_system/issues/16>`_ from IntelligentRoboticsLabs/pddl_parser_rename
  Rename pddl_parser
* Rename pddl_parser
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Merge pull request `#15 <https://github.com/IntelligentRoboticsLabs/ros2_planning_system/issues/15>`_ from IntelligentRoboticsLabs/example_rename
  Rename example. Small bug in timeouts
* Linting
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Rename example. Small bug in timeouts
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Merge pull request `#12 <https://github.com/IntelligentRoboticsLabs/ros2_planning_system/issues/12>`_ from IntelligentRoboticsLabs/actions_composition
  Define rate dynamically
* Define rate dynamically
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Packages.xml description
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Adding documentation
  Signed-off-by: Francisco Martin Rico <fmartin@gsyc.urjc.es>
* Setting CI
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Setting CI
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Setting CI
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Setting CI
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* onActivate and onFinished methods for Action Clients
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* First functional version complete
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Execute actions independiently. Example
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Change to lowercasegit
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Executor initial version
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Contributors: Francisco Martin Rico
