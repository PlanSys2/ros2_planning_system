
#pragma once

#include "plansys2_pddl_parser/Type.h"

namespace parser { namespace pddl {

class EitherType : public Type {

public:

	EitherType( const std::string & s )
		: Type( s ) {}

	EitherType( const EitherType * t )
		: Type( t ) {}
	
	std::string getName() const {
		std::string out = "either";
		for ( unsigned i = 0; i < subtypes.size(); ++i )
			out += "_" + subtypes[i]->getName();
		return out;
	}

	void PDDLPrint( std::ostream & s ) const override {}

	Type * copy() {
		return new EitherType( this );
	}

};

} } // namespaces
