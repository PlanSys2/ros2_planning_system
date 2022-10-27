
#include "plansys2_pddl_parser/Domain.h"

namespace parser { namespace pddl {

void Forall::PDDLPrint( std::ostream & s, unsigned indent, const TokenStruct< std::string > & ts, const Domain & d ) const {
	tabindent( s, indent );
	s << "( forall\n";

	TokenStruct< std::string > fstruct( ts );

	tabindent( s, indent + 1 );
	printParams( 0, s, fstruct, d );

	if ( cond ) cond->PDDLPrint( s, indent + 1, fstruct, d );
	else {
		tabindent( s, indent + 1 );
		s << "()";
	}

	s << "\n";
	tabindent( s, indent );
	s << ")";
}

plansys2_msgs::msg::Node::SharedPtr Forall::getTree( plansys2_msgs::msg::Tree & tree, const Domain & d, const std::vector<std::string> & replace ) const {
//    throw UnsupportedConstruct("Forall");

  plansys2_msgs::msg::Node::SharedPtr node = std::make_shared<plansys2_msgs::msg::Node>();
  node->node_type = plansys2_msgs::msg::Node::FOR_ALL;
  node->node_id = tree.nodes.size();
  tree.nodes.push_back(*node);

  for (unsigned j = 0; j < params.size(); j++) {
    plansys2_msgs::msg::Param param;
    auto p  = params[j];
    param.name = "?" + d.types[p]->getName() + std::to_string(j);
    param.type = d.types[p]->name;
    d.types[p]->getSubTypesNames(param.sub_types);
    node->parameters.push_back(param);
  }

  plansys2_msgs::msg::Node::SharedPtr child = cond->getTree(tree, d, {});
  node->children.push_back(child->node_id);

  tree.nodes[node->node_id] = *node;
  return node;

}

void Forall::parse( Stringreader & f, TokenStruct< std::string > & ts, Domain & d ) {
	f.next();
	f.assert_token( "(" );

	TokenStruct< std::string > fs = f.parseTypedList( true, d.types );
	params = d.convertTypes( fs.types );

	TokenStruct< std::string > fstruct( ts );
	fstruct.append( fs );

	f.next();
	f.assert_token( "(" );
	if ( f.getChar() != ')' ) {
		cond = d.createCondition( f );
		cond->parse( f, fstruct, d );
	}
	else ++f.c;

	f.next();
	f.assert_token( ")" );
}

} } // namespaces
