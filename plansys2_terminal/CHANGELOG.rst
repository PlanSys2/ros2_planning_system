^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package plansys2_terminal
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.0 (2021-07-04)
------------------
* Fix compile issues for galactic
* Compile for ROS2 Galactic
* Contributors: Francisco Martín Rico, Jonatan Olofsson, bjnjo

1.0.10 (2021-07-03)
-------------------
* Revert wrong directory move
* Remove node parameter in client constructors
* Enabling code coverage of plansys2_terminal::Terminal node init functions.
* Adding unit tests for new functions in plansys2_terminal.
* Add ability to read in pddl problem files to plansys2 and a new AddProblem service to the plansys2_problem_expert.
  Adding problem_file node parameter to plansys2_problem_expert to load a single problem file at launch.
* ros2-plan-msg: Passing plan to executor to add further separation between plan creation and plan execution.
* Linting
* Fix set predicates with no arguments
* Suggestion for `#118 <https://github.com/IntelligentRoboticsLabs/ros2_planning_system/issues/118>`_
* pddl-tree-messages: Performing some minor cleanup.
* pddl-tree-messages: Using explicit specifier for single parameter constructors in plansys2_core/Types.hpp.
* pddl-tree-messages: Replacing user access function calls with shorter versions where possible.
* pddl-tree-messages: Updating addInstance and removeInstance calls to use helper classes.
* pddl-tree-messages: Merging upstream master and resolving conflicts.
* pddl-tree-messages: Using ROS messages to define the PDDL construct trees.
* Fix tests and linting
* Update version
* Contributors: Alexander Xydes, Francisco Martín Rico, Josh Zapf

1.0.9 (2021-03-15)
------------------

1.0.8 (2021-03-12)
------------------
* Action execution refactoring
* Add support for numeric conditions and effects.
* Monitorization info
* fix minor typo
* Adding actor checker in terminal
* Remove debugging traces
* Fix the call of the get_problem_instance service
* Contributors: Fabrice Larribe, Francisco Martin Rico, Josh Zapf

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
* Contributors: Francisco Martin Rico


0.0.8 (2020-07-18)
------------------
* Add BT support
* Contributors: Francisco Martin Rico

0.0.7 (2020-03-26)
------------------
* Fix warning in last cmake versions
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Fix spaces in command line
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Contributors: Francisco Martín Rico
0.0.6 (2020-03-23)
------------------
* Terminal completion functionality
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Add multi domain
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
* Add readline dependency
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Contributors: Francisco Martín Rico
0.0.2 (2020-01-08)
------------------
* Packages.xml description
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Improved stdin read
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* First functional version complete
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Execute actions independiently. Example
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Change to lowercasegit
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Executor initial version
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* First version of planner complete
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Update notification in problem
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Problem expert complete with terminal support
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Problem expert client and node
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Domain types and messages changed
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Predicate Tree and types changed
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Planning terminal and domain clients
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Contributors: Francisco Martín Rico