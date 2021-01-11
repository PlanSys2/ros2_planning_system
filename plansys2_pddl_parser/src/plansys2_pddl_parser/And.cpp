
#include "plansys2_pddl_parser/Domain.h"

namespace parser { namespace pddl {

void And::PDDLPrint( std::ostream & s, unsigned indent, const TokenStruct< std::string > & ts, const Domain & d ) const {
	tabindent( s, indent );
	s << "( and\n";
	for ( unsigned i = 0; i < conds.size(); ++i ) {
		conds[i]->PDDLPrint( s, indent + 1, ts, d );
		s << "\n";
	}
	tabindent( s, indent );
	s << ")";
}

std::shared_ptr<tree::TreeNode> And::PDDLTree( const Domain & d ) const {
    std::shared_ptr<tree::AndNode> tree = std::make_shared<tree::AndNode>();
    for ( unsigned i = 0; i < conds.size(); ++i ) {
        tree->ops.push_back( conds[i]->PDDLTree( d ) );
    }
    return tree;
}

void And::parse( Stringreader & f, TokenStruct< std::string > & ts, Domain & d ) {
	for ( f.next(); f.getChar() != ')'; f.next() ) {
		f.assert_token( "(" );
		Condition * condition = d.createCondition( f );
		condition->parse( f, ts, d );
		conds.push_back( condition );
	}
	++f.c;
}

} } // namespaces
