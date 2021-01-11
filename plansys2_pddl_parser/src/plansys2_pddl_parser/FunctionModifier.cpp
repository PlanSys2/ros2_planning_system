
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

std::shared_ptr<tree::TreeNode> FunctionModifier::PDDLTree( const Domain & d ) const {
    std::shared_ptr<tree::FunctionModifierNode> tree = std::make_shared<tree::FunctionModifierNode>();
    tree->modifier_type = tree::getFunModType( name );
    if (modifiedGround) {
        tree->ops.push_back( modifiedGround->PDDLTree( d ) );
    }
    else {
        std::cerr << "function modifier for total-cost not supported" << std::endl;
    }
    tree->ops.push_back( modifierExpr->PDDLTree( d ) );
    return tree;
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
