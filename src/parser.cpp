#include <iostream>

#include "pddl_parser/Instance.h"

using namespace parser::pddl;

int main( int argc, char *argv[] ) {
	if ( argc < 3 ) {
		std::cout << "Usage: parser <domain.pddl> <task.pddl>\n";
		exit( 1 );
	}

	// Read multiagent domain and instance
	Domain domain( argv[1] );
	Instance instance( domain, argv[2] );

	std::cout << domain << std::endl;
	std::cout << instance << std::endl;
}