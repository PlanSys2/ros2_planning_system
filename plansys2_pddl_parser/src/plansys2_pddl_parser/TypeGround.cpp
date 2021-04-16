
#include "plansys2_pddl_parser/Domain.h"

namespace parser { namespace pddl {

void TypeGround::PDDLPrint( std::ostream & s, unsigned indent, const TokenStruct< std::string > & ts, const Domain & d ) const {
	tabindent( s, indent );
	s << "( " << name;
	for ( unsigned i = 0; i < params.size(); ++i )
		s << " " << d.types[lifted->params[i]]->object( params[i] ).first;
	s << " )";
}

plansys2_msgs::msg::Node::SharedPtr TypeGround::getTree( plansys2_msgs::msg::Tree & tree, const Domain & d, const std::vector<std::string> & replace ) const {
    return Ground::getTree(tree, d, replace);
}

void TypeGround::insert( Domain & d, const StringVec & v ) {
	params.resize( lifted->params.size() );
	for ( unsigned i = 0; i < lifted->params.size(); ++i ) {
		std::pair< bool, unsigned > p = d.types[lifted->params[i]]->parseObject( v[i] );
		if ( p.first ) params[i] = p.second;
		else {
			std::pair< bool, int > q = d.types[lifted->params[i]]->parseConstant( v[i] );
			if ( q.first ) params[i] = q.second;
			else {
				std::cerr << "Unknown object " << v[i] << "\n";
				std::exit( 1 );
			}
		}
	}
}

void TypeGround::parse( Stringreader & f, TokenStruct< std::string > & ts, Domain & d ) {
	f.next();
	params.resize( lifted->params.size() );
	for ( unsigned i = 0; i < lifted->params.size(); ++i, f.next() ) {
		std::string s = f.getToken();
		std::pair< bool, unsigned > p = d.types[lifted->params[i]]->parseObject( s );
		if ( p.first ) params[i] = p.second;
		else {
			std::pair< bool, int > q = d.types[lifted->params[i]]->parseConstant( s );
			if ( q.first ) params[i] = q.second;
			else f.tokenExit( s );
		}
	}
	f.assert_token( ")" );
}

} } // namespaces
