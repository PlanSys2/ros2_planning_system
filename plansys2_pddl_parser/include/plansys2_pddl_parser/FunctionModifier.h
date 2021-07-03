
#pragma once

#include "plansys2_msgs/msg/node.hpp"
#include "plansys2_msgs/msg/tree.hpp"

#include "plansys2_pddl_parser/Ground.h"
#include "plansys2_pddl_parser/Condition.h"
#include "plansys2_pddl_parser/Function.h"
#include "plansys2_pddl_parser/Expression.h"


namespace parser { namespace pddl {

class FunctionModifier : public Condition {

public:

	std::string name;

	Ground * modifiedGround;  // if null -> total-cost
	Expression * modifierExpr;  // the expression by which we increment/decrement

	FunctionModifier( const std::string& name, int val = 1 );

	FunctionModifier( const std::string& name, Function * f, const IntVec & p = IntVec() );

	FunctionModifier( const std::string& name, const FunctionModifier * i, Domain & d );

	~FunctionModifier() {
		if ( modifiedGround ) delete modifiedGround;
		if ( modifierExpr ) delete modifierExpr;
	}

	void print( std::ostream & s ) const {
		s << name << " ";
		if ( modifiedGround ) modifiedGround->print( s );
		if ( modifierExpr ) modifierExpr->print( s );
		s << "\n";
	}

	void PDDLPrint( std::ostream & s, unsigned indent, const TokenStruct< std::string > & ts, const Domain & d ) const override;

	plansys2_msgs::msg::Node::SharedPtr getTree( plansys2_msgs::msg::Tree & tree, const Domain & d, const std::vector<std::string> & replace = {} ) const override;

	void parse( Stringreader & f, TokenStruct< std::string > & ts, Domain & d );

	void addParams( int m, unsigned n ) {}
};

class Assign : public FunctionModifier {

public:

    Assign( int val = 1 ) : FunctionModifier( "assign", val ) { }

    Assign( Function * f, const IntVec & p = IntVec() ) : FunctionModifier( "assign", f, p ) { }

    Assign( const FunctionModifier * i, Domain & d ) : FunctionModifier( "assign", i, d ) { }

    Condition * copy( Domain & d ) {
        return new Assign( this, d );
    }
};

class Decrease : public FunctionModifier {

public:

	Decrease( int val = 1 ) : FunctionModifier( "decrease", val ) { }

	Decrease( Function * f, const IntVec & p = IntVec() ) : FunctionModifier( "decrease", f, p ) { }

	Decrease( const FunctionModifier * i, Domain & d ) : FunctionModifier( "decrease", i, d ) { }

	Condition * copy( Domain & d ) {
		return new Decrease( this, d );
	}
};

class Increase : public FunctionModifier {

public:

	Increase( int val = 1 ) : FunctionModifier( "increase", val ) { }

	Increase( Function * f, const IntVec & p = IntVec() ) : FunctionModifier( "increase", f, p ) { }

	Increase( const FunctionModifier * i, Domain & d ) : FunctionModifier( "increase", i, d ) { }

	Condition * copy( Domain & d ) {
		return new Increase( this, d );
	}
};

} } // namespaces
