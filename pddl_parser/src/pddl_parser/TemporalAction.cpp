
#include "pddl_parser/Instance.h"

namespace parser { namespace pddl {

Expression * TemporalAction::parseDuration( Stringreader & f, TokenStruct< std::string > & ts, Domain & d ) {
	return createExpression( f, ts, d );
}

void TemporalAction::printCondition( std::ostream & s, const TokenStruct< std::string > & ts, const Domain & d,
									 const std::string & t, And * a ) const {
	for ( unsigned i = 0; a && i < a->conds.size(); ++i ) {
		s << "\t\t( " << t << " ";
		a->conds[i]->PDDLPrint( s, 0, ts, d );
		s << " )\n";
	}
}

void TemporalAction::PDDLPrint( std::ostream & s, unsigned indent, const TokenStruct< std::string > & ts, const Domain & d ) const {
	s << "( :DURATIVE-ACTION " << name << "\n";

	s << "  :PARAMETERS ";

	TokenStruct< std::string > astruct;

	printParams( 0, s, astruct, d );

	s << "  :DURATION ( = ?DURATION ";
	if ( durationExpr ) durationExpr->PDDLPrint( s, 0, astruct, d );
	else s << "1";
	s << " )\n";

	s << "  :CONDITION\n";
	s << "\t( AND\n";
	printCondition( s, astruct, d, "AT START", (And *)pre );
	printCondition( s, astruct, d, "OVER ALL", pre_o );
	printCondition( s, astruct, d, "AT END", pre_e );
	s << "\t)\n";

	s << "  :EFFECT\n";
	s << "\t( AND\n";
	printCondition( s, astruct, d, "AT START", (And *)eff );
	printCondition( s, astruct, d, "AT END", eff_e );
	s << "\t)\n";

	s << ")\n";
}

void TemporalAction::parseCondition( Stringreader & f, TokenStruct< std::string > & ts, Domain & d, And * a ) {
	f.next();
	f.assert_token( "(" );
	Condition * c = d.createCondition( f );
	c->parse( f, ts, d );
	a->conds.push_back( c );
}

void TemporalAction::parse( Stringreader & f, TokenStruct< std::string > & ts, Domain & d ) {
	f.next();
	f.assert_token( ":PARAMETERS" );
	f.assert_token( "(" );

	TokenStruct< std::string > astruct = f.parseTypedList( true, d.types );
	params = d.convertTypes( astruct.types );

	f.next();
	f.assert_token( ":DURATION" );
	f.assert_token( "(" );
	f.assert_token( "=" );
	f.assert_token( "?DURATION" );
	durationExpr = parseDuration( f, astruct, d );
	f.next();
	f.assert_token( ")" );

	f.next();
	f.assert_token( ":" );
	std::string s = f.getToken();
	if ( s == "CONDITION" ) {
		pre = new And;
		pre_o = new And;
		pre_e = new And;
		f.next();
		f.assert_token( "(" );
		if ( f.getChar() != ')' ) {
			s = f.getToken();
			if ( s == "AND" ) {
				for ( f.next(); f.getChar() != ')'; f.next() ) {
					f.assert_token( "(" );
					s = f.getToken();
					f.next();
					std::string t = f.getToken();

					if ( s == "AT" && t == "START" )
						parseCondition( f, astruct, d, (And *)pre );
					else if ( s == "OVER" && t == "ALL" )
						parseCondition( f, astruct, d, pre_o );
					else if ( s == "AT" && t == "END" )
						parseCondition( f, astruct, d, pre_e );
					else f.tokenExit( s + " " + t );

					f.next();
					f.assert_token( ")" );
				}
				++f.c;
			}
			else {
				f.next();
				std::string t = f.getToken();

				if ( s == "AT" && t == "START" )
					parseCondition( f, astruct, d, (And *)pre );
				else if ( s == "OVER" && t == "ALL" )
					parseCondition( f, astruct, d, pre_o );
				else if ( s == "AT" && t == "END" )
					parseCondition( f, astruct, d, pre_e );
				else f.tokenExit( s + " " + t );

				f.next();
				f.assert_token( ")" );
			}
		}
		else ++f.c;

		f.next();
		f.assert_token( ":" );
		s = f.getToken();
	}
	if ( s != "EFFECT" ) f.tokenExit( s );

	f.next();
	f.assert_token( "(" );
	if ( f.getChar() != ')' ) {
		eff = new And;
		eff_e = new And;

		s = f.getToken();
		if ( s == "AND" ) {
			for ( f.next(); f.getChar() != ')'; f.next() ) {
				f.assert_token( "(" );
				s = f.getToken();
				f.next();
				std::string t = f.getToken();

				if ( s == "AT" && t == "START" )
					parseCondition( f, astruct, d, (And *)eff );
				else if ( s == "AT" && t == "END" )
					parseCondition( f, astruct, d, eff_e );
				else f.tokenExit( s + " " + t );

				f.next();
				f.assert_token( ")" );
			}
			++f.c;
		}
		else {
			f.next();
			std::string t = f.getToken();

			if ( s == "AT" && t == "START" )
				parseCondition( f, astruct, d, (And *)eff );
			else if ( s == "AT" && t == "END" )
				parseCondition( f, astruct, d, eff_e );
			else f.tokenExit( s + " " + t );

			f.next();
			f.assert_token( ")" );
		}
	}
	else ++f.c;

	f.next();
	f.assert_token( ")" );
}

GroundVec TemporalAction::preconsStart() {
	return getGroundsFromCondition( pre, false );
}

GroundVec TemporalAction::preconsOverall() {
	return getGroundsFromCondition( pre_o, false );
}

GroundVec TemporalAction::preconsEnd() {
	return getGroundsFromCondition( pre_e, false );
}

CondVec TemporalAction::endEffects() {
	return getSubconditionsFromCondition( eff_e );
}

GroundVec TemporalAction::addEndEffects() {
	return getGroundsFromCondition( eff_e, false );
}

GroundVec TemporalAction::deleteEndEffects() {
	return getGroundsFromCondition( eff_e, true );
}

} } // namespaces
