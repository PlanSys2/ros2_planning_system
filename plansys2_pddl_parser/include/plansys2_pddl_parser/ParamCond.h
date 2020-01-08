
#pragma once

#include "plansys2_pddl_parser/Condition.h"
#include "plansys2_pddl_parser/Basic.h"

namespace parser { namespace pddl {


// This is necessary for adl
using ::operator<<;

class ParamCond : public Condition {

public:
	std::string name;
	IntVec params;

	ParamCond() {}

	ParamCond( const std::string & s, const IntVec & p = IntVec() )
		: name( s ), params( p ) {}

	ParamCond( const ParamCond * c )
		: name( c->name ), params( c->params ) {}

	std::string c_str() const {
		return name;
	}

	void print( std::ostream & stream ) const {
		stream << name << params << "\n";
	}

	void printParams( unsigned first, std::ostream & s, TokenStruct< std::string > & ts, const Domain & d ) const;
};

typedef std::vector< ParamCond * > ParamCondVec;

} } // namespaces
