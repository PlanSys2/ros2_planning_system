int16 REQUEST=1
int16 RESPONSE=2
int16 CONFIRM=3
int16 REJECT=4
int16 FEEDBACK=5
int16 FINISH=6
int16 CANCEL=7

int16 type
string node_id
# Set by the requester in REQUEST and carried by every message of that request, so each
# message reaches the request it belongs to
string request_id

string action
string[] arguments

bool success
float32 completion
string status

# REQUEST: the duration the plan gives the action, in seconds (0 for a non-durative one)
float64 planned_duration
