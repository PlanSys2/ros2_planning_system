
#include "plansys2_pddl_parser/Domain.h"

namespace parser { namespace pddl {

FunctionModifier::FunctionModifier( const std::string& name, int val )
	: name( name ), modifiedGround( 0 ), modifierExpr( new ValueExpression( val ) ) {}

FunctionModifier::FunctionModifier( const std::string& name, Function * f, const IntVec & p )
	: name( name ), modifiedGround( 0 ), modifierExpr( new FunctionExpression( new Ground( f, p ) ) ) {}

FunctionModifier::FunctionModifier( const std::string& name, const FunctionModifier * i, Domain & d )
	: name( name )
{
	if ( i->modifiedGround ) {
		modifiedGround = dynamic_cast< Ground * >( i->modifiedGround->copy( d ) );
	}
	else modifiedGround = 0;

	if ( i->modifierExpr ) {
		modifierExpr = dynamic_cast< Expression * >( i->modifierExpr->copy( d ) );
	}
	else modifierExpr = 0;
}

void FunctionModifier::PDDLPrint( std::ostream & s, unsigned indent, const TokenStruct< std::string > & ts, const Domain & d ) const {
	tabindent( s, indent );
	s << "( " << name << " ";

	if ( modifiedGround ) {
		modifiedGround->PDDLPrint( s, 0, ts, d );
	}
	else {
		s << "( total-cost )";
	}

	s << " ";
	modifierExpr->PDDLPrint( s, 0, ts, d );

	s << " )";
}

plansys2_msgs::msg::Node::SharedPtr FunctionModifier::getTree( plansys2_msgs::msg::Tree & tree, const Domain & d, const std::vector<std::string> & replace ) const {
    plansys2_msgs::msg::Node::SharedPtr node = std::make_shared<plansys2_msgs::msg::Node>();
    node->node_type = plansys2_msgs::msg::Node::FUNCTION_MODIFIER;
    node->modifier_type = getFunModType(name);
    node->node_id = tree.nodes.size();
    tree.nodes.push_back(*node);

    if (modifiedGround) {
        plansys2_msgs::msg::Node::SharedPtr child = modifiedGround->getTree(tree, d, replace);
        tree.nodes[node->node_id].children.push_back(child->node_id);
    }
    else {
        std::cerr << "function modifier for total-cost not supported" << std::endl;
    }

    plansys2_msgs::msg::Node::SharedPtr child = modifierExpr->getTree(tree, d, replace);
    tree.nodes[node->node_id].children.push_back(child->node_id);

    return node;
}

void FunctionModifier::parse( Stringreader & f, TokenStruct< std::string > & ts, Domain & d ) {
	f.next();

	f.assert_token( "(" );

	std::string increasedFunction = f.getToken();
	if ( increasedFunction == "total-cost" ) {
 		f.next();
		f.assert_token( ")" );
	}
	else {
		modifiedGround = new Ground( d.funcs.get( increasedFunction ) );
		modifiedGround->parse( f, ts, d );
	}

	f.next();

	modifierExpr = createExpression( f, ts, d );

	f.next();
	f.assert_token( ")" );
}

} } // namespaces
