
#include "plansys2_pddl_parser/Domain.h"

namespace parser { namespace pddl {

Ground::Ground( const Ground * g, Domain & d )
	: ParamCond( g ), lifted( d.preds.get( g->name ) ) {}

void Ground::PDDLPrint( std::ostream & s, unsigned indent, const TokenStruct< std::string > & ts, const Domain & d ) const {
	tabindent( s, indent );
	s << "( " << name;
	for ( unsigned i = 0; i < params.size(); ++i ) {
		if ( ts.size() && params[i] >= 0 && (unsigned)params[i] < ts.size() ) s << " " << ts[params[i]];
		else if (params[i] >= 0 && (unsigned)params[i] >= ts.size()) s << " ?" << params[i]; 
		else s << " " << d.types[lifted->params[i]]->object( params[i] ).first;
	}
	s << " )";
}

std::shared_ptr<tree::TreeNode> Ground::PDDLTree( const Domain & d ) const {
    if ( d.funcs.index( name ) >= 0) {
        std::shared_ptr<tree::FunctionNode> tree = std::make_shared<tree::FunctionNode>();
        tree->function_.name = name;
        for ( unsigned i = 0; i < params.size(); ++i ) {
            tree::Param param;
            param.name = "?" + std::to_string(params[i]);
            param.type = d.types[lifted->params[i]]->name;
            tree->function_.parameters.push_back(param);
        }
        return tree;
    }
    else {
        std::shared_ptr<tree::PredicateNode> tree = std::make_shared<tree::PredicateNode>();
        tree->predicate_.name = name;
        for ( unsigned i = 0; i < params.size(); ++i ) {
            tree::Param param;
            param.name = "?" + std::to_string(params[i]);
            param.type = d.types[lifted->params[i]]->name;
            tree->predicate_.parameters.push_back(param);
        }
        return tree;
    }
}

void Ground::parse( Stringreader & f, TokenStruct< std::string > & ts, Domain & d ) {
	f.next();
	params.resize( lifted->params.size() );
	for ( unsigned i = 0; i < lifted->params.size(); ++i, f.next() ) {
		std::string s = f.getToken();
		int k = ts.index( s );
		if ( k >= 0 ) params[i] = k;
		else {
			std::pair< bool, int > p = d.types[lifted->params[i]]->parseConstant( s );
			if ( p.first ) params[i] = p.second;
			else f.tokenExit( s );
		}
	}
	f.assert_token( ")" );
}

} } // namespaces
