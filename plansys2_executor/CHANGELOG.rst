^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package plansys2_executor
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Merge remote-tracking branch 'origin/humble-devel'
* Merge pull request `#252 <https://github.com/PlanSys2/ros2_planning_system/issues/252>`_ from PlanSys2/check_at_end
  Check at end reqs in bt builder
* Check at end reqs in bt builder
* Merge pull request `#251 <https://github.com/PlanSys2/ros2_planning_system/issues/251>`_ from PlanSys2/fix_bt_node
  Fix bt node
* Change MultiThreaded for SingleThreaded in CI failing tests
* Change double quotes for simple ones (linter)
* Insert in blackboard the action ROS 2 Node
* Merge pull request `#247 <https://github.com/PlanSys2/ros2_planning_system/issues/247>`_ from jjzapf/standalone-bt-builder
  Standalone BT Builder Service
* Fixing cpplint warning.
* Fixing cpplint warnings.
* Checking for self-referencing edges in STNBTBuilder. Adding standalone compute_bt service.
* Merge remote-tracking branch 'origin/master' into fix_goal_structure_issue_205
* Merge pull request `#240 <https://github.com/PlanSys2/ros2_planning_system/issues/240>`_ from jjzapf/bt-builder-plugins
  New BT Builder and Plugin Interface
* bt-builder-plugins: Setting default BT builder plugin to SimpleBTBuilder.
* bt-builder-plugins: Creating BT builder plugin interface. Moving current BT builder to plugin named SimpleBTBuilder. Adding new and improved STN-based BT builder plugin named STNBTBuilder.
* Merge remote-tracking branch 'origin/master' into fix_goal_structure_issue_205
* Merge remote-tracking branch 'upstream/master'
* Merge branch 'IntelligentRoboticsLabs:master' into master
* Contributors: Andrianov Roman, Francisco Martín Rico, Josh Zapf, Marco Roveri, Splinter1984

2.0.9 (2022-07-10)
------------------
* Humble Upgrade
* Fix possible bug https://github.com/ros2/rclcpp/issues/1968
* Expose lifecyclemanager timeout as a parameter, other misc fixes/features
* Linters are explicit
* Add ctor to ExecutorClient for varying node name
* Fix uninitialized variable in ExecutorClient
* Contributors: Francisco Martín Rico, Jake Keller

2.0.8 (2022-05-04)
------------------

2.0.7 (2022-05-04)
------------------

2.0.6 (2022-05-03)
------------------

2.0.5 (2022-05-03)
------------------

2.0.4 (2022-05-03)
------------------
* Fix version
* Merge pull request `#223 <https://github.com/IntelligentRoboticsLabs/ros2_planning_system/issues/223>`_ from IntelligentRoboticsLabs/fix_threads_buildfarm
  Fix threads buildfarm
* Fix ROS2 Buildfarm error due to Threads
* Merge pull request `#217 <https://github.com/IntelligentRoboticsLabs/ros2_planning_system/issues/217>`_ from jjzapf/check-action-finished
  Check action finished
* check-action-finished: Putting leftover requirement check after state update in get_graph function.
* check-action-finished: Use distinct names for temporary predicate/function variables inside of while loop.
* check-action-finished: Reverting previous changes. Adding checks to WaitAction node to verify that action has finished.
* check-action-finished: Fixing problem_expert.wait_overall_req_test in plansys2_executor/bt_node_test.cpp.
* check-action-finished: Do not declare failure in check_overall_req_node if action has already finished.
* Merge pull request `#209 <https://github.com/IntelligentRoboticsLabs/ros2_planning_system/issues/209>`_ from sarcasticnature/master
  Improvements on recent BtActionNode changes
* Merge pull request `#216 <https://github.com/IntelligentRoboticsLabs/ros2_planning_system/issues/216>`_ from jjzapf/action-graph-bug-fix
  Plan-to-Action Graph Bug Fix
* action-graph-bug-fix: Not applying at end effects when testing if actions can be run in parallel. Fixing bug in prune_backwards function.
* Update unit tests to match changes
* Remove extra do_work() call in on_activate()
  do_work() may call the function finish(), which will fail to deactivate
  the node if it is still in the on_activate callback.
* Merge branch 'IntelligentRoboticsLabs:master' into master
* Merge pull request `#208 <https://github.com/IntelligentRoboticsLabs/ros2_planning_system/issues/208>`_ from xydesa/plan-dotgraph-bug
  Plan dotgraph color bug
* Merge branch 'IntelligentRoboticsLabs:master' into master
* Removing unused variable.
* More accurately getting the status of an action by including the action's start time in the index for coloring the dotgraph.
* Contributors: Alexander Xydes, Francisco Martín Rico, Jake Keller, Josh Zapf

2.0.3 (2022-04-03)
------------------

2.0.2 (2022-04-03)
------------------
* Use apply and check method to create action graph
* plan-to-action-graph-mod: Using apply and check method to search for causal links.
* generalize-is-parallelizable: Cleaning up comments a bit.
* generalize-is-parallelizable: Checking for contradictions at any point in time in the is_parallelizable function.
* generalize-is-parallelizable: Improving human readability.
* generalize-is-parallelizable: Applying ament uncrustify.
* generalize-is-parallelizable: Generalizing the is_parallelizable function in BTBuilder so that the functions will also be checked.
* action-graph-test: Adding unit test to verify action graph generation.
* action-graph-fix: Using std::list to create action graph rather than …
* action-graph-fix: Shortening line lengths to <100 characters. Applying ament uncrustify.
* action-graph-fix: Checking for existing link before adding one when creating action graph.
* Check all reqs and effect for roots
* action-graph-fix: Setting executor_test TIMEOUT value to 300.
* Add options to run command in plansys2 Terminal
* Change runtime failures to BT::NodeStatus::FAILURE, add logging info to ExecutorClient
* Add options to run command
* action-graph-fix: Using std::list to create action graph rather than std::set. A std::set does not maintain insertion order, whereas a std::list does. Maintaining insertion order guarantees that graph traversal follows the same path as graph creation.
* action-graph-test: Adding unit test to verify action graph generation.
* Add Status recency in performers
* Remove unnecessary node pointer
* Merge branch 'IntelligentRoboticsLabs:master' into master
* Logger tool - performers and plan
* Logger tool - knowledge, info and action hub
* Elevate failure logging from INFO to WARN/ERROR
* Add action failure details to ExecutorClient
* Contributors: Francisco Martín Rico, Jake Keller, Josh Zapf

2.0.1 (2022-02-03)
------------------
* Update deprecated APIs in launchers and parameters
* Update launcher param names
* Improve debug info
* remove-invalid-goals: Removing invalid goals when instances are removed. Keep track of update time in problem expert.
* Merge galactic-devel
* Contributors: Francisco Martín Rico, Josh Zapf

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
