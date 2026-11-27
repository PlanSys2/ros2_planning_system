string[] instances
string[] predicates
# With their values: (= (battery r1) 10.5), (= (pos r1) kitchen)
string[] functions
string goal
string metric
# PDDL3: what the plan keeps, (always (safe)); empty if none
string constraints
# HDDL: what is to be done, as tasks; empty in PDDL problems
plansys2_msgs/TaskNetwork task_network
