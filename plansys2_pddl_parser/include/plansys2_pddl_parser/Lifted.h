
#pragma once

#include "plansys2_pddl_parser/ParamCond.h"

namespace parser { namespace pddl {

class Lifted : public ParamCond {

public:

	Lifted() {}

	Lifted( const std::string & s )
		: ParamCond( s ) {}

	Lifted( const ParamCond * c )
		: ParamCond( c ) {}

	void PDDLPrint( std::ostream & s, unsigned indent, const TokenStruct< std::string > & ts, const Domain & d ) const override;

	std::shared_ptr<tree::TreeNode> PDDLTree( const Domain & d ) const override;

	void parse( Stringreader & f, TokenStruct< std::string > & ts, Domain & d );

	void addParams( int m, unsigned n ) {}

	Condition * copy( Domain & d ) {
		return new Lifted( this );
	}

};

typedef std::vector< Lifted * > LiftedVec;

} } // namespaces
