# FAQ

## 1 How to debug errors in PDDL?

Working with PDDL is difficult. If the PDDL domain is not well designed, or predicates or instances are missing, it is impossible to generate a plan. Plansys2 will notify you, but it is difficult to debug and solve the problem.

When it is required to generate a plan, Plansys2 generates a file with the domain, `/tmp/domain.pddl` and another with the problem `/tmp/domain.pddl`. In addition, the output of the plan reseller (popf) is saved in /tmp/plan.pddl.

It is possible to execute the plan solver in isolation using the command:

```ros2 run popf popf /tmp/domain.pddl /tmp/problem.pddl```

Use `ros2 run popf popf -h` for more help.