
#include "plansys2_pddl_parser/Domain.h"

namespace parser { namespace pddl {

void Lifted::PDDLPrint( std::ostream & s, unsigned indent, const TokenStruct< std::string > & ts, const Domain & d ) const {
	tabindent( s, indent );
	s << "( " << name;
	for ( unsigned i = 0; i < params.size(); ++i ) {
		if ( ts.size() ) s << ts[i];
		else s << " ?" << d.types[params[i]]->getName() << i;
		if ( d.typed ) s << " - " << d.types[params[i]]->name;
	}
	s << " )";
}

std::shared_ptr<tree::TreeNode> Lifted::PDDLTree( const Domain & d ) const {
    throw UnsupportedConstruct("Lifted");
}

void Lifted::parse( Stringreader & f, TokenStruct< std::string > & ts, Domain & d ) {
	TokenStruct< std::string > lstruct = f.parseTypedList( true, d.types );
	params = d.convertTypes( lstruct.types );
}


} } // namespaces
