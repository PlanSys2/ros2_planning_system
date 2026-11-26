# A task of a hierarchical (HTN, HDDL) plan and how it was decomposed. PDDL plans have none.
#
# The first task of Plan.tasks is the initial task network: its `task` and `method` are empty.

# Unique in the plan
string id
# "(name arg1 arg2 ...)"; empty for the initial task network
string task
# The method that decomposes it; empty for the initial task network
string method
# Its subtasks, in the order the method writes them: the id of a task of Plan.tasks, or
# "item:<i>" for the action Plan.items[i]
string[] subtasks
# Pairs, one after the other: subtasks[ordering[2k]] ends before subtasks[ordering[2k+1]]
# starts. Subtasks that no pair orders may run in any order, or at once
uint32[] ordering
# What must hold when the method is applied (at its start, for a durative method), in PDDL with
# the values of its parameters; empty if the method has no precondition
string precondition
