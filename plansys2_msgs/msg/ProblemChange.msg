# One change of a problem, for problem_expert/apply_changes

uint8 ADD_INSTANCE=1
uint8 REMOVE_INSTANCE=2     # also removes its facts and the parts of the goal on it
uint8 ADD_PREDICATE=3
uint8 REMOVE_PREDICATE=4
uint8 SET_FUNCTION=5        # added if it was not in the problem
uint8 REMOVE_FUNCTION=6

uint8 type
# ADD_INSTANCE and REMOVE_INSTANCE
plansys2_msgs/Param instance
# Predicates and functions
plansys2_msgs/Node node
