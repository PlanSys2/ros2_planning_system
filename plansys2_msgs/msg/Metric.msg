# What the planner optimizes: (:metric minimize|maximize <expression>)
uint8 MINIMIZE = 0
uint8 MAXIMIZE = 1

uint8 optimization

# Numeric expression; (total-time) is a function node named total-time. Empty: no metric
plansys2_msgs/Tree tree
