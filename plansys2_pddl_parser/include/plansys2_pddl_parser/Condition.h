
#pragma once

#include "plansys2_pddl_parser/Basic.h"
#include "plansys2_pddl_parser/Stringreader.h"
#include "plansys2_pddl_parser/Type.h"

namespace parser { namespace pddl {

class Condition {

public:

	virtual ~Condition() {}

	virtual void print( std::ostream & stream ) const = 0;

	virtual void PDDLPrint( std::ostream & s, unsigned indent, const TokenStruct< std::string > & ts, const Domain & d ) const = 0;

	virtual void parse( Stringreader & f, TokenStruct< std::string > & ts, Domain & d ) = 0;

	virtual void addParams( int m, unsigned n ) = 0;

	virtual Condition * copy( Domain & d ) = 0;
};

inline std::ostream & operator<<( std::ostream & stream, const Condition * c ) {
	c->print( stream );
	return stream;
}

typedef std::vector< Condition * > CondVec;

} } // namespaces
