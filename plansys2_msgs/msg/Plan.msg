plansys2_msgs/PlanItem[] items

# When the problem it solves was read: its times count from then, as those of its timed facts
# and of what changes by itself. Zero: from when it starts
builtin_interfaces/Time stamp

# PDDL+: the plan ends at this time, after its last action, so that processes and events act
bool has_end
float64 end

# The value of the problem metric for this plan, when the planner gives it
bool has_cost
float64 cost
