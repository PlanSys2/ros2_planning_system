
#pragma once

#include "plansys2_msgs/msg/node.hpp"
#include "plansys2_msgs/msg/tree.hpp"

#include "plansys2_pddl_parser/ParamCond.h"

namespace parser { namespace pddl {

class Imply : public ParamCond {

public:
	Condition * cond;

	Imply()
		: cond( 0 ) {}

	Imply( const Imply * f, Domain & d )
		: ParamCond( f ), cond( 0 ) {
		if ( f->cond ) cond = f->cond->copy( d );
	}

	~Imply() {
		if ( cond ) delete cond;
	}

	void print( std::ostream & s ) const {
		s << "Imply" << params << ":\n";
		if ( cond ) cond->print( s );
	}

	void PDDLPrint( std::ostream & s, unsigned indent, const TokenStruct< std::string > & ts, const Domain & d ) const override;

	plansys2_msgs::msg::Node::SharedPtr getTree( plansys2_msgs::msg::Tree & tree, const Domain & d, const std::vector<std::string> & replace = {} ) const override;

	void parse( Stringreader & f, TokenStruct< std::string > & ts, Domain & d );

	void addParams( int m, unsigned n ) {
		cond->addParams( m, n );
	}

	Condition * copy( Domain & d ) {
		return new Imply( this, d );
	}

};

} } // namespaces
