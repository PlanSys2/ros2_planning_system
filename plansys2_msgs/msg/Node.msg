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

# Expression types
uint8 COMP_EQ = 10
uint8 COMP_GE = 11
uint8 COMP_GT = 12
uint8 COMP_LE = 13
uint8 COMP_LT = 14
uint8 ARITH_MULT = 15
uint8 ARITH_DIV = 16
uint8 ARITH_ADD = 17
uint8 ARITH_SUB = 18

# Function modifier types
uint8 ASSIGN = 19
uint8 INCREASE = 20
uint8 DECREASE = 21
uint8 SCALE_UP = 22
uint8 SCALE_DOWN = 23

uint8 node_type
uint8 expression_type
uint8 modifier_type

uint32 node_id
uint32[] children

string name
plansys2_msgs/Param[] parameters
float64 value
bool negate
