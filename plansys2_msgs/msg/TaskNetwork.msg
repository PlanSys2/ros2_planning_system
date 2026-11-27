# The initial task network of a hierarchical (HDDL) problem: what is to be done, as tasks. In
# PDDL problems it is empty.

# Its tasks, "(name arg1 arg2 ...)": abstract tasks of the domain, or actions
string[] tasks
# The label of each task, used by `ordering` in the problem text (empty: it is given one)
string[] ids
# Pairs, one after the other: tasks[ordering[2k]] ends before tasks[ordering[2k+1]] starts.
# Tasks that no pair orders may be done in any order, or at once
uint32[] ordering
