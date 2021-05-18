
#pragma once

#include "plansys2_msgs/msg/node.hpp"
#include "plansys2_msgs/msg/tree.hpp"

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

	plansys2_msgs::msg::Node::SharedPtr getTree( plansys2_msgs::msg::Tree & tree, const Domain & d, const std::vector<std::string> & replace = {} ) const override;

	void parse( Stringreader & f, TokenStruct< std::string > & ts, Domain & d );
	
};

} } // namespaces
