
#include "plansys2_pddl_parser/Domain.h"

namespace parser { namespace pddl {

void And::PDDLPrint( std::ostream & s, unsigned indent, const TokenStruct< std::string > & ts, const Domain & d ) const {
	tabindent( s, indent );
  if (conds.size() > 1){
    s << "( and\n";
  }
	for ( unsigned i = 0; i < conds.size(); ++i ) {
		conds[i]->PDDLPrint( s, indent + 1, ts, d );
		s << "\n";
	}
	tabindent( s, indent );
  if (conds.size() > 1){
    s << ")\n";
  }
}

plansys2_msgs::msg::Node::SharedPtr And::getTree( plansys2_msgs::msg::Tree & tree, const Domain & d, const std::vector<std::string> & replace ) const {
    plansys2_msgs::msg::Node::SharedPtr node = std::make_shared<plansys2_msgs::msg::Node>();
    node->node_type = plansys2_msgs::msg::Node::AND;


    for ( unsigned i = 0; i < conds.size(); ++i) {
        plansys2_msgs::msg::Node::SharedPtr child = conds[i]->getTree(tree, d, replace);
        node->children.push_back(child->node_id);
    }

    node->node_id = tree.nodes.size();
    tree.nodes.push_back(*node);
    return node;
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
