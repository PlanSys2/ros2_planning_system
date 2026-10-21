^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package plansys2_support_py
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

4.0.0 (2026-10-21)
------------------
* One version for Humble, Jazzy, Kilted and Lyrical (`#509 <https://github.com/PlanSys2/ros2_planning_system/pull/509>`_)
* Robust PlanSys2: one node per component, clients that wait, retry and see a lost plan (`#507 <https://github.com/PlanSys2/ros2_planning_system/pull/507>`_)
* Execute everything Kairos plans: conditional effects, durations, timed literals, object fluents, recursive derived predicates, PDDL+ and constraints (`#504 <https://github.com/PlanSys2/ros2_planning_system/pull/504>`_)
* Fixes in the executor, the build, the tests and the CI (`#502 <https://github.com/PlanSys2/ros2_planning_system/pull/502>`_)
* Fixes in the Python support, the terminal, and the rqt and TUI tools (`#500 <https://github.com/PlanSys2/ros2_planning_system/pull/500>`_)
* ActionExecutor and performers: one request, its own messages (`#488 <https://github.com/PlanSys2/ros2_planning_system/pull/488>`_)
* Problem metric from Python, the terminal and the TUI (`#463 <https://github.com/PlanSys2/ros2_planning_system/pull/463>`_)
* forall and imply in conditions, goals and effects (`#462 <https://github.com/PlanSys2/ros2_planning_system/pull/462>`_)
* Tests discover only nodes on this host (`#455 <https://github.com/PlanSys2/ros2_planning_system/pull/455>`_)
* Performers, executor clients and the TUI notice when PlanSys2 goes down (`#448 <https://github.com/PlanSys2/ros2_planning_system/pull/448>`_)
* Consistent executor goal handling: cancels, replaced goals and result codes (`#439 <https://github.com/PlanSys2/ros2_planning_system/pull/439>`_)
* Update domain during execution (`#414 <https://github.com/PlanSys2/ros2_planning_system/pull/414>`_)
* Contributors: Francisco Martín Rico

3.1.0 (2026-07-28)
------------------
* Added some tests
* Added planner in python
* Convert to fully python package
* Added execute_plan to executor
* Added parser, format and lint
* Update logs and parser
* Implement DomainExpertClient, ExecutorClient, PlannerClient, PlannerClient and ProblemExpertClient
* Contributors: Alberto Tudela, Eshan Savla, Francisco Martín Rico, Gustavo, Gustavo Rezende

3.0.0 (2025-06-06)
------------------
* Support action performers in Python
* Contributors: Francisco Martín Rico, Marco Roveri
