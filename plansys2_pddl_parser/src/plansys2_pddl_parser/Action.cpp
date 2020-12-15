
#include "plansys2_pddl_parser/Domain.h"

namespace parser { namespace pddl {

void Action::PDDLPrint( std::ostream & s, unsigned indent, const TokenStruct< std::string > & ts, const Domain & d ) const {
	s << "( :action " << name << "\n";

	s << "  :parameters ";

	TokenStruct< std::string > astruct;

	printParams( 0, s, astruct, d );

	s << "  :precondition\n";
	if ( pre ) pre->PDDLPrint( s, 1, astruct, d );
	else s << "\t()";
	s << "\n";

	s << "  :effect\n";
	if ( eff ) eff->PDDLPrint( s, 1, astruct, d );
	else s << "\t()";
	s << "\n";

	s << ")\n";
}

std::shared_ptr<tree::TreeNode> Action::PDDLTree( const Domain & d ) const {
    std::shared_ptr<tree::ActionNode> tree = std::make_shared<tree::ActionNode>();
    if ( pre ) {
        tree->pre.push_back( pre->PDDLTree( d ) );
    }
    if ( eff ) {
        tree->eff.push_back( eff->PDDLTree( d ) );
    }
    return tree;
}

void Action::parseConditions( Stringreader & f, TokenStruct< std::string > & ts, Domain & d ) {
	f.next();
	f.assert_token( ":" );
	std::string s = f.getToken();
	if ( s == "precondition" ) {
		f.next();
		f.assert_token( "(" );
		if ( f.getChar() != ')' ) {
			pre = d.createCondition( f );
			pre->parse( f, ts, d );
		}
		else ++f.c;

		f.next();
		f.assert_token( ":" );
		s = f.getToken();
	}
	if ( s != "effect" ) f.tokenExit( s );

	f.next();
	f.assert_token( "(" );
	if ( f.getChar() != ')' ) {
		eff = d.createCondition( f );
		eff->parse( f, ts, d );
	}
	else ++f.c;
	f.next();
	f.assert_token( ")" );
}

void Action::parse( Stringreader & f, TokenStruct< std::string > & ts, Domain & d ) {
	f.next();
	f.assert_token( ":parameters" );
	f.assert_token( "(" );

	TokenStruct< std::string > astruct = f.parseTypedList( true, d.types );
	params = d.convertTypes( astruct.types );

	parseConditions( f, astruct, d );
}

CondVec Action::precons() {
	return getSubconditionsFromCondition( pre );
}

CondVec Action::effects() {
	return getSubconditionsFromCondition( eff );
}

GroundVec Action::addEffects() {
	return getGroundsFromCondition( eff, false );
}

GroundVec Action::deleteEffects() {
	return getGroundsFromCondition( eff, true );
}

CondVec Action::getSubconditionsFromCondition( Condition * c ) {
	And * a = dynamic_cast< And * >( c );
	if ( a ) return a->conds;

	CondVec subconds;
	if ( c ) subconds.push_back( c );
	return subconds;
}

GroundVec Action::getGroundsFromCondition( Condition * c, bool neg ) {
	GroundVec grounds;
	And * a = dynamic_cast< And * >( c );
	for ( unsigned i = 0; a && i < a->conds.size(); ++i ) {
		if ( neg ) {
			Not * n = dynamic_cast< Not * >( a->conds[i] );
			if ( n ) grounds.push_back( n->cond );
		}
		else {
			Ground * g = dynamic_cast< Ground * >( a->conds[i] );
			if ( g ) grounds.push_back( g );
		}
	}

	if ( neg ) {
		Not * n = dynamic_cast< Not * >( c );
		if ( n ) grounds.push_back( n->cond );
	}
	else {
		Ground * g = dynamic_cast< Ground * >( c );
		if ( g ) grounds.push_back( g );
	}

	return grounds;
}

} } // namespaces
