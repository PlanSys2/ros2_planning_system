int16 SETUP_REQUEST=1
int16 SETUP_RESPONSE=2
int16 REQUEST=3
int16 RESPONSE=4
int16 CONFIRM=5
int16 REJECT=6
int16 FEEDBACK=7
int16 FINISH=8
int16 CANCEL=9

int16 type
string node_id

string action
string[] arguments
builtin_interfaces/Duration duration
float32 duration_overrun_percentage

bool success
float32 completion
string status
