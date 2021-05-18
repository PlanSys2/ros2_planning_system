
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

plansys2_msgs::msg::Node::SharedPtr Action::getTree( plansys2_msgs::msg::Tree & tree, const Domain & d, const std::vector<std::string> & replace ) const {
    plansys2_msgs::msg::Node::SharedPtr node = std::make_shared<plansys2_msgs::msg::Node>();
    node->node_type = plansys2_msgs::msg::Node::ACTION;
    node->node_id = tree.nodes.size();
    tree.nodes.push_back(*node);

    if (pre) {
        plansys2_msgs::msg::Node::SharedPtr child = pre->getTree(tree, d, replace);
        tree.nodes[node->node_id].children.push_back(child->node_id);
    }

    if (eff) {
        plansys2_msgs::msg::Node::SharedPtr child = eff->getTree(tree, d, replace);
        tree.nodes[node->node_id].children.push_back(child->node_id);
    }

    return node;
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
