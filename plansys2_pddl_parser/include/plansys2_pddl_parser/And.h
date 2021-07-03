
#pragma once

#include "plansys2_msgs/msg/node.hpp"
#include "plansys2_msgs/msg/tree.hpp"

#include "plansys2_pddl_parser/Condition.h"

namespace parser { namespace pddl {

class And : public Condition {

public:
	CondVec conds;

	And() = default;

	And( const And * a, Domain & d ) {
		for ( unsigned i = 0; i < a->conds.size(); ++i )
			conds.push_back( a->conds[i]->copy( d ) );
	}

	~And() {
		for ( unsigned i = 0; i < conds.size(); ++i )
			delete conds[i];
	}

	void print( std::ostream & s ) const {
		for ( unsigned i = 0; i < conds.size(); ++i )
			conds[i]->print( s );
	}

	void PDDLPrint( std::ostream & s, unsigned indent, const TokenStruct< std::string > & ts, const Domain & d ) const override;

	plansys2_msgs::msg::Node::SharedPtr getTree( plansys2_msgs::msg::Tree & tree, const Domain & d, const std::vector<std::string> & replace = {} ) const override;

	void parse( Stringreader & f, TokenStruct< std::string > & ts, Domain & d );

	void add( Condition * cond ) {
		conds.push_back( cond );
	}

	void addParams( int m, unsigned n ) {
		for ( unsigned i = 0; i < conds.size(); ++i )
			conds[i]->addParams( m, n );
	}

	Condition * copy( Domain & d ) {
		return new And( this, d );
	}

};

} } // namespaces
