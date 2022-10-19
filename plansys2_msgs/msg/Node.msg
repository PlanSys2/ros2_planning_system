# Unknown type
uint8 UNKNOWN = 0

# Node types
uint8 AND = 1
uint8 OR = 2
uint8 NOT = 3
uint8 ACTION = 4
uint8 PREDICATE = 5
uint8 FUNCTION = 6
uint8 EXPRESSION = 7
uint8 FUNCTION_MODIFIER = 8
uint8 NUMBER = 9
uint8 NOT_KNOWN = 10

# Expression types
uint8 COMP_EQ = 11
uint8 COMP_GE = 12
uint8 COMP_GT = 13
uint8 COMP_LE = 14
uint8 COMP_LT = 15
uint8 ARITH_MULT = 16
uint8 ARITH_DIV = 17
uint8 ARITH_ADD = 18
uint8 ARITH_SUB = 19

# Function modifier types
uint8 ASSIGN = 20
uint8 INCREASE = 21
uint8 DECREASE = 22
uint8 SCALE_UP = 23
uint8 SCALE_DOWN = 24

uint8 node_type
uint8 expression_type
uint8 modifier_type

uint32 node_id
uint32[] children

string name
plansys2_msgs/Param[] parameters
float64 value
bool negate
