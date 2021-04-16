
#pragma once

#include "plansys2_msgs/msg/node.hpp"
#include "plansys2_msgs/msg/tree.hpp"

#include "plansys2_pddl_parser/Ground.h"

namespace parser { namespace pddl {

class Action : public ParamCond {

public:

	Condition *pre, *eff;

	Action( const std::string & s )
		: ParamCond( s ), pre( 0 ), eff( 0 ) {}

	Action( ParamCond * c )
		: ParamCond( c ), pre( 0 ), eff( 0 ) {}

	Action( const Action * a, Domain & d )
		: ParamCond( a ), pre( 0 ), eff( 0 ) {
		if ( a->pre ) pre = a->pre->copy( d );
		if ( a->eff ) eff = a->eff->copy( d );
	}

	virtual ~Action() {
		if ( pre ) delete pre;
		if ( eff ) delete eff;
	}

	void print( std::ostream & s ) const {
		s << name << params << "\n";
		s << "Pre: " << pre;
		if ( eff ) s << "Eff: " << eff;
	}

	virtual double duration() {
		return 1;
	}

	void PDDLPrint( std::ostream & s, unsigned indent, const TokenStruct< std::string > & ts, const Domain & d ) const override;

	plansys2_msgs::msg::Node::SharedPtr getTree( plansys2_msgs::msg::Tree & tree, const Domain & d, const std::vector<std::string> & replace = {} ) const override;

	void parseConditions( Stringreader & f, TokenStruct< std::string > & ts, Domain & d );

	void parse( Stringreader & f, TokenStruct< std::string > & ts, Domain & d );

	void addParams( int m, unsigned n ) {}

	void addParams( const IntVec & v ) {
		if ( pre ) pre->addParams( params.size(), v.size() );
		if ( eff ) eff->addParams( params.size(), v.size() );
		params.insert( params.end(), v.begin(), v.end() );
	}

	Condition * copy( Domain & d ) {
		return new Action( this, d );
	}

	CondVec precons();

	CondVec effects();

	GroundVec addEffects();

	GroundVec deleteEffects();

protected:

	CondVec getSubconditionsFromCondition( Condition * c );

	GroundVec getGroundsFromCondition( Condition * c, bool neg );
};

typedef std::vector< Action * > ActionVec;

} } // namespaces
