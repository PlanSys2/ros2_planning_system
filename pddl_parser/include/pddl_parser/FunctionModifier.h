
#pragma once

#include "pddl_parser/Ground.h"
#include "pddl_parser/Condition.h"
#include "pddl_parser/Function.h"
#include "pddl_parser/Expression.h"


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

	void parse( Stringreader & f, TokenStruct< std::string > & ts, Domain & d );

	void addParams( int m, unsigned n ) {}
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
