# What the executor does while a plan runs, and why: one message per event, in order

uint8 PLAN_START=1  # detail: how it runs (timed or as soon as possible), its actions
uint8 WAIT=2        # an action waits: detail says for what (its time, another action, a condition)
uint8 PERFORM=3     # a performer takes the action: detail says which
uint8 START=4       # its start happened: its at start effects are applied
uint8 END=5         # its end happened: its at end effects are applied
uint8 SHIFT=6       # the rest of the plan goes later: detail says by how much, and why
uint8 FAILURE=7     # detail says what failed, and why
uint8 VIOLATION=8   # a constraint is violated: detail says which
uint8 REPLAN=9      # a new plan replaces the running one: detail says what goes on
uint8 PLAN_END=10   # detail says how it ended

uint8 type
# Consecutive within an executor, also across plans: the order of events of the same instant
uint64 sequence
builtin_interfaces/Time stamp
# Seconds since the plan started, shifted by the late actions
float64 plan_time

# The action, as the executor names it: "(move r1 kitchen hall):3000"; empty for the plan
string action
# START and END: when the plan said it happens, in its time
float64 planned_time
# How much later the plan goes by then, as the late actions before shifted it: late is
# plan_time - planned_time - shift
float64 shift
string detail
