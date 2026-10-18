string name
plansys2_msgs/Param[] parameters

plansys2_msgs/Tree at_start_requirements
plansys2_msgs/Tree over_all_requirements
plansys2_msgs/Tree at_end_requirements
plansys2_msgs/Tree at_start_effects
plansys2_msgs/Tree at_end_effects

# While it runs (PDDL 2.1): (and (increase (f) (* #t rate))...); #t is the time it has acted
plansys2_msgs/Tree continuous_effects

# Its duration: (and (<op> ?duration <expression>)...), as in the domain. ?duration, also in the
# conditions and effects, is the duration a plan gives the action
plansys2_msgs/Tree duration
