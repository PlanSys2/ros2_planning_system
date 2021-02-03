
#include "plansys2_pddl_parser/Domain.h"

namespace parser { namespace pddl {

void When::PDDLPrint( std::ostream & s, unsigned indent, const TokenStruct< std::string > & ts, const Domain & d ) const {
	tabindent( s, indent );
	s << "( when\n";
	if ( pars ) pars->PDDLPrint( s, indent + 1, ts, d );
	else {
		tabindent( s, indent + 1 );
		s << "()";
	}
	s << "\n";
	if ( cond ) cond->PDDLPrint( s, indent + 1, ts, d );
	else {
		tabindent( s, indent + 1 );
		s << "()";
	}
	s << "\n";
	tabindent( s, indent );
	s << ")";
}

std::shared_ptr<tree::TreeNode> When::PDDLTree( const Domain & d ) const {
    throw UnsupportedConstruct("When");
}

void When::parse( Stringreader & f, TokenStruct< std::string > & ts, Domain & d ) {
	f.next();
	f.assert_token( "(" );
	if ( f.getChar() != ')' ) {
		pars = d.createCondition( f );
		pars->parse( f, ts, d );
	}
	else ++f.c;

	f.next();
	f.assert_token( "(" );
	if ( f.getChar() != ')' ) {
		cond = d.createCondition( f );
		cond->parse( f, ts, d );
	}
	else ++f.c;

	f.next();
	f.assert_token( ")" );
}

} } // namespaces
