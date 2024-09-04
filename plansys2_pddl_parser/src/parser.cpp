#include <iostream>
#include <string>
#include <fstream>
#include <streambuf>


#include "plansys2_pddl_parser/Instance.h"

using namespace parser::pddl;

int main( int argc, char *argv[] ) {
	if ( argc < 3 ) {
		std::cout << "Usage: parser <domain.pddl> <task.pddl>\n";
		exit( 1 );
	}
	std::ifstream domain_ifs(argv[1]);
	std::string domain_str((
		std::istreambuf_iterator<char>(domain_ifs)),
    std::istreambuf_iterator<char>());

	std::ifstream instance_ifs(argv[2]);
	std::string instance_str((
		std::istreambuf_iterator<char>(instance_ifs)),
    std::istreambuf_iterator<char>());

	// Read multiagent domain and instance
	Domain domain( domain_str );
	Instance instance( domain, instance_str );

	std::cout << domain << std::endl;
	std::cout << instance << std::endl;
}