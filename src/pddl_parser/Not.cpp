
#include "pddl_parser/Domain.h"

namespace parser { namespace pddl {

void Not::PDDLPrint( std::ostream & s, unsigned indent, const TokenStruct< std::string > & ts, const Domain & d ) const {
	tabindent( s, indent );
	s << "( NOT ";
	if ( cond ) cond->PDDLPrint( s, 0, ts, d );
	s << " )";
}

void Not::parse( Filereader & f, TokenStruct< std::string > & ts, Domain & d ) {
	f.next();
	f.assert_token( "(" );

	cond = dynamic_cast< Ground * >( d.createCondition( f ) );

	if ( !cond ) {
		f.tokenExit( f.getToken() );
	}

	cond->parse( f, ts, d );

	f.next();
	f.assert_token( ")" );
}


} } // namespaces
