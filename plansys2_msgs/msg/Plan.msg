plansys2_msgs/PlanItem[] items

# PDDL+: the plan ends at this time, after its last action, so that processes and events act
bool has_end
float64 end

# The value of the problem metric for this plan, when the planner gives it
bool has_cost
float64 cost
