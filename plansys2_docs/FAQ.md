# FAQ

## 1. How to debug errors in PDDL?

Working with PDDL is difficult. If the PDDL domain is not well designed, or predicates or instances are missing, it is impossible to generate a plan. Plansys2 will notify you, but it is difficult to debug and solve the problem.

Kairos, the default plan solver, reports errors in the domain and the problem with their line and column in the log of the planner and of the domain expert. To work on them outside PlanSys2, save the domain (`get domain` in the terminal, or the service `/domain_expert/get_domain`) and the problem (the service `/problem_expert/get_problem`) and run Kairos on them:

``` shell
kairos check domain.pddl problem.pddl   # errors and warnings, and the PDDL features used
kairos domain.pddl problem.pddl -v      # the plan, with the progress of the search
```

If the goal cannot be reached, Kairos says which part of it no action can achieve.

With POPF as plan solver, PlanSys2 writes the domain to `/tmp/${namespace}/domain.pddl`, the problem to `/tmp/${namespace}/problem.pddl` and the output of POPF to `/tmp/${namespace}/plan.pddl`, and POPF can be run in isolation with `ros2 run popf popf /tmp/${namespace}/domain.pddl /tmp/${namespace}/problem.pddl`.
