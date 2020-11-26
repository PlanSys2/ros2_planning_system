
#pragma once
#include <map>
#include "plansys2_pddl_parser/Basic.h"
/*
	T is either a pointer or a string
	A token structure stores pointers but does not delete them!
*/

namespace parser { namespace pddl {

typedef std::map< std::string, unsigned > TokenMap;

template <typename T>
inline std::string getName( const T & t ) {
	return t->name;
}

template <>
inline std::string getName<std::string>( const std::string & s ) {
	return s;
}

template <typename T>
class TokenStruct {

public:

	std::vector< T > tokens;
	TokenMap tokenMap;

	// represents the types of a typed list
	StringVec types;

	TokenStruct() {}

	TokenStruct( const TokenStruct & ts )
		: tokens( ts.tokens ), tokenMap( ts.tokenMap ), types( ts.types ) {}

	void append( const TokenStruct & ts ) {
		for ( unsigned i = 0; i < ts.size(); ++i )
			insert( ts[i] );
		types.insert( types.end(), ts.types.begin(), ts.types.end() );
	}

	unsigned size() const {
		return tokens.size();
	}

	T & operator[]( std::size_t i ) {
		return tokens[i];
	}

	const T & operator[]( std::size_t i ) const {
		return tokens[i];
	}

	void clear() {
		for ( unsigned i = 0; i < size(); ++i )
			delete tokens[i];

		tokens.clear();
		tokenMap.clear();
	}

	unsigned insert( const T & t ) {
		TokenMap::iterator i = tokenMap.insert( tokenMap.begin(), std::make_pair( getName( t ), size() ) );
		tokens.push_back( t );
		return i->second;
	}

	int index( const std::string & s ) const {
		TokenMap::const_iterator i = tokenMap.find( s );
		return i == tokenMap.end() ? -1 : i->second;
	}

	T get( const std::string & s ) const {
		return tokens[index( s )];
	}

};

} } // namespaces
