
#pragma once

#include "plansys2_pddl_parser/Ground.h"

namespace parser { namespace pddl {

class Equals : public Ground {

public:

	Equals( const IntVec & p = IntVec() )
		: Ground( "=", p ) {}

	Equals( const Equals * e )
		: Ground( "=", e->params ) {}

	void PDDLPrint( std::ostream & s, unsigned indent, const TokenStruct< std::string > & ts, const Domain & d ) const override;

    std::shared_ptr<tree::TreeNode> PDDLTree( const Domain & d ) const override;

	void parse( Stringreader & f, TokenStruct< std::string > & ts, Domain & d );

	Condition * copy( Domain & d ) {
		return new Equals( this );
	}

};

} } // namespaces
