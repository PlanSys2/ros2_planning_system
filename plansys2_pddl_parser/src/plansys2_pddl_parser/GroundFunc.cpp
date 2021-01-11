
#include "plansys2_pddl_parser/Domain.h"

namespace parser { namespace pddl {

template <>
void GroundFunc<double>::PDDLPrint( std::ostream & s, unsigned indent, const TokenStruct< std::string > & ts, const Domain & d ) const {
	tabindent( s, indent );
	s << "( = ";
	TypeGround::PDDLPrint( s, 0, ts, d );
	s << " " << ( double )value << " )";
}

template <>
void GroundFunc<int>::PDDLPrint( std::ostream & s, unsigned indent, const TokenStruct< std::string > & ts, const Domain & d ) const {
	tabindent( s, indent );
	s << "( = ";
	TypeGround::PDDLPrint( s, 0, ts, d );
	s << " " << d.types[((Function *)lifted)->returnType]->object( value ) << " )";
}

template <>
std::shared_ptr<tree::TreeNode> GroundFunc<double>::PDDLTree( const Domain & d ) const {
    throw UnsupportedConstruct("GroundFunc");
}

template <>
std::shared_ptr<tree::TreeNode> GroundFunc<int>::PDDLTree( const Domain & d ) const {
    throw UnsupportedConstruct("GroundFunc");
}

template <>
void GroundFunc<double>::parse( Stringreader & f, TokenStruct< std::string > & ts, Domain & d ) {
	TypeGround::parse( f, ts, d );
	
	f.next();
	std::string s = f.getToken();
	std::istringstream i( s );
	if ( !( i >> value ) ) f.tokenExit( s );

	f.next();
	f.assert_token( ")" );
}

template <>
void GroundFunc<int>::parse( Stringreader & f, TokenStruct< std::string > & ts, Domain & d ) {
	TypeGround::parse( f, ts, d );
	
	f.next();
	std::string s = f.getToken();
	std::pair< bool, unsigned > p = d.types[((Function *)lifted)->returnType]->parseObject( s );
	if ( p.first ) value = p.second;
	else {
		std::pair< bool, int > q = d.types[((Function *)lifted)->returnType]->parseConstant( s );
		if ( q.first ) value = q.second;
		else f.tokenExit( s );
	}

	f.next();
	f.assert_token( ")" );
}

} } // namespaces
