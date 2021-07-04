^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package plansys2_bt_actions
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.0 (2021-07-04)
------------------

1.0.10 (2021-07-03)
-------------------
* Fix tests
* Fix rate conversions
* Reduce debug output
* Update version
* Contributors: Francisco Martín Rico

1.0.9 (2021-03-15)
------------------

1.0.8 (2021-03-12)
------------------
* Moving zmq publisher creation to on_activate in BTAction, resetting the publisher in on_deactivate.
* Action execution refactoring
* Adding zeromq-based groot monitoring of plansys2 behaviortree actions. Ports are not specified by default to keep two different actions from accidentally using the same ports.
* Add support for numeric conditions and effects.
* Monitorization info
* Improving BTActions
* Change 'move' action name
* Contributors: Alexander Xydes, Fabrice Larribe, Francisco Martin Rico, Josh Zapf

1.0.7 (2021-01-04)
------------------
* Making explicit dependencies
* Contributors: Francisco Martín Rico
1.0.6 (2020-12-29)
------------------

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
* Contributors: Francisco Martin Rico


0.0.8 (2020-07-18)
------------------
* Boost:optional
* Contributors: Francisco Martin Rico

0.0.7 (2020-03-26)
------------------
* Fix warning in last cmake versions
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Contributors: Francisco Martín Rico
0.0.6 (2020-03-23)
------------------
* Run in separate namespaces. Monolothic node
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

0.0.2 (2020-01-08)
------------------
* Merge pull request `#16 <https://github.com/IntelligentRoboticsLabs/ros2_planning_system/issues/16>`_ from IntelligentRoboticsLabs/pddl_parser_rename
  Rename pddl_parser
* Rename pddl_parser
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Merge pull request `#8 <https://github.com/IntelligentRoboticsLabs/ros2_planning_system/issues/8>`_ from IntelligentRoboticsLabs/patrol_example
  Patrol example
* Patrol example
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
* Execute actions independiently. Example
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Change to lowercasegit
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* First version of planner complete
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Update notification in problem
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Problem expert complete with terminal support
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Problem expert client and node
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Goals in problem generation
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* ProblemExpert local complete
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Using shred_ptr. First commit Problem
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Predicate Tree and types changed
  Signed-off-by: Francisco Martin Rico <fmrico@gmail.com>
* Contributors: Francisco Martin Rico
