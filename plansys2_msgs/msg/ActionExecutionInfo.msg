int8 NOT_EXECUTED=1
int8 EXECUTING=2
int8 FAILED=3
int8 SUCCEEDED=4
int8 CANCELLED=5

int8 status

builtin_interfaces/Time start_stamp
builtin_interfaces/Time status_stamp

string action_full_name

string action
string[] arguments

# predicted duration of the action from the planner
builtin_interfaces/Duration duration

float32 completion
string message_status
