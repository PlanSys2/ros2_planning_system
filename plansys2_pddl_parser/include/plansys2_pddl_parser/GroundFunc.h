
#pragma once

#include "plansys2_pddl_parser/TypeGround.h"

namespace parser { namespace pddl {

template < typename T >
class GroundFunc : public TypeGround {

public:

	T value;

	GroundFunc()
		: TypeGround(), value( 0 ) {}

	GroundFunc( Lifted * l, const T & val = T( 0 ) )
		: TypeGround( l ), value( val ) {}

	void PDDLPrint( std::ostream & s, unsigned indent, const TokenStruct< std::string > & ts, const Domain & d ) const override;

	std::shared_ptr<tree::TreeNode> PDDLTree( const Domain & d ) const override;

	void parse( Stringreader & f, TokenStruct< std::string > & ts, Domain & d );

};

} } // namespaces
