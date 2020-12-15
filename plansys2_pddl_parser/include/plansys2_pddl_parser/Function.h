
#pragma once

#include "plansys2_pddl_parser/Lifted.h"

namespace parser { namespace pddl {

class Function : public Lifted {

public:

	int returnType;

	Function()
		: Lifted(), returnType( -1 ) {}

	Function( const std::string & s, int type = -1 )
		: Lifted( s ), returnType( type ) {}

	Function( const ParamCond * c )
		: Lifted( c ), returnType( -1 ) {}

	void PDDLPrint( std::ostream & s, unsigned indent, const TokenStruct< std::string > & ts, const Domain & d ) const override;

	std::shared_ptr<tree::TreeNode> PDDLTree( const Domain & d ) const override;

	void parse( Stringreader & f, TokenStruct< std::string > & ts, Domain & d );
	
};

} } // namespaces
