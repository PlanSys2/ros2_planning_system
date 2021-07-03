
#pragma once

#include <stdexcept>

#include "plansys2_msgs/msg/node.hpp"
#include "plansys2_msgs/msg/tree.hpp"

#include "plansys2_pddl_parser/Basic.h"
#include "plansys2_pddl_parser/Stringreader.h"
#include "plansys2_pddl_parser/Utils.h"
#include "plansys2_pddl_parser/Type.h"

namespace parser { namespace pddl {

class UnsupportedConstruct : public std::runtime_error {
public:
    UnsupportedConstruct(const std::string& construct) : std::runtime_error(construct +  " is not currently supported by plansys2") {}
};

class Condition {

public:

	virtual ~Condition() {}

	virtual void print( std::ostream & stream ) const = 0;

	virtual void PDDLPrint( std::ostream & s, unsigned indent, const TokenStruct< std::string > & ts, const Domain & d ) const = 0;

	virtual plansys2_msgs::msg::Node::SharedPtr getTree( plansys2_msgs::msg::Tree & tree, const Domain & d, const std::vector<std::string> & replace = {} ) const = 0;

	virtual void parse( Stringreader & f, TokenStruct< std::string > & ts, Domain & d ) = 0;

	virtual void addParams( int m, unsigned n ) = 0;

	virtual Condition * copy( Domain & d ) = 0;
};

inline std::ostream & operator<<( std::ostream & stream, const Condition * c ) {
	c->print( stream );
	return stream;
}

typedef std::vector< Condition * > CondVec;

} } // namespaces
