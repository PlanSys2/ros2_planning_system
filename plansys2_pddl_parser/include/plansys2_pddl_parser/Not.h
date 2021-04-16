
#pragma once

#include "plansys2_msgs/msg/node.hpp"
#include "plansys2_msgs/msg/tree.hpp"

#include "plansys2_pddl_parser/Ground.h"

namespace parser { namespace pddl {

class Not : public Condition {

public:

	Ground * cond;

	Not()
		: cond( 0 ) {}

	Not( Ground * g )
		: cond( g ) {}

	Not( const Not * n, Domain & d )
		: cond( 0 ) {
		if ( n->cond ) cond = ( Ground * )n->cond->copy( d );
	}

	~Not() {
		if ( cond ) delete cond;
	}

	void print( std::ostream & s ) const {
		s << "not ";
		if ( cond ) cond->print( s );
	}

	void PDDLPrint( std::ostream & s, unsigned indent, const TokenStruct< std::string > & ts, const Domain & d ) const override;

	plansys2_msgs::msg::Node::SharedPtr getTree( plansys2_msgs::msg::Tree & tree, const Domain & d, const std::vector<std::string> & replace = {} ) const override;

	void parse( Stringreader & f, TokenStruct< std::string > & ts, Domain & d );

	void addParams( int m, unsigned n ) {
		cond->addParams( m, n );
	}

	Condition * copy( Domain & d ) {
		return new Not( this, d );
	}

};

} } // namespaces
