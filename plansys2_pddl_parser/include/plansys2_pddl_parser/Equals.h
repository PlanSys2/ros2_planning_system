
#pragma once

#include "plansys2_msgs/msg/node.hpp"
#include "plansys2_msgs/msg/tree.hpp"

#include "plansys2_pddl_parser/Ground.h"

namespace parser { namespace pddl {

class Equals : public Ground {

public:

	Equals( const IntVec & p = IntVec() )
		: Ground( "=", p ) {}

	Equals( const Equals * e )
		: Ground( "=", e->params ) {}

	void PDDLPrint( std::ostream & s, unsigned indent, const TokenStruct< std::string > & ts, const Domain & d ) const override;

	plansys2_msgs::msg::Node::SharedPtr getTree( plansys2_msgs::msg::Tree & tree, const Domain & d, const std::vector<std::string> & replace = {} ) const override;

	void parse( Stringreader & f, TokenStruct< std::string > & ts, Domain & d );

	Condition * copy( Domain & d ) {
		return new Equals( this );
	}

};

} } // namespaces
