# A fact change made by the state updater, and why: what missions replan on
# (problem_expert/update_notify is also published by the dynamics)

uint8 FUNCTION=1
uint8 PREDICATE=2
uint8 INSTANCE=3     # fact: its name; holds: created (true) or removed (false)

builtin_interfaces/Time stamp
string rule
uint8 kind
# (energy), (sensor_ok imu), or an instance name
string fact
float64 value
# For object fluents, (= (pos r1) kitchen): the new object
string object
bool holds
# false if the previous value is unknown (e.g. the first value of a function)
bool has_previous
float64 previous_value
string previous_object
bool previous_holds
# e.g. battery: 95 → 82, deviation 13 ≥ 2
string reason
# false in a dry run, or when the problem expert rejected it (retried later)
bool written
