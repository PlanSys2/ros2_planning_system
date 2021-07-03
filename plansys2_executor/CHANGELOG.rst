^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package plansys2_executor
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.9 (2021-03-15)
------------------
* Disable boost in tests
* Contributors: Francisco Martin Rico

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
* Contributors: Francisco Martin Rico

1.0.6 (2020-12-29)
------------------
* Disable boost functions
* Contributors: Francisco Martin Rico

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
* Contributors: Fabrice Larribe, Francisco Martin Rico, Francisco Martín Rico, f269858

1.0.1 (2020-07-19)
------------------

1.0.0 (2020-07-19)
------------------
* Foxy initial version
* Boost:optional
* Support for BT actions
* Contributors: Francisco Martin Rico, Francisco Martín Rico

0.0.8 (2020-07-18)
------------------
* Add BT support
* Contributors: Francisco Martin Rico, Francisco Martín Rico

0.0.7 (2020-03-26)
------------------
* ActionExecutorClient is cascade_lifecycle
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Contributors: Francisco Martin Rico, Francisco Martín Rico

0.0.6 (2020-03-23)
------------------
* Run in separate namespaces. Monolothic node
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Contributors: Francisco Martin Rico, Francisco Martín Rico

0.0.5 (2020-01-12)
------------------

0.0.4 (2020-01-09)
------------------
* Adding missing action dependencies
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Contributors: Francisco Martin Rico

0.0.3 (2020-01-09)
------------------
* Add popf dependency
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Contributors: Francisco Martin Rico

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
* Contributors: Francisco Martin Rico, Francisco Martín Rico
