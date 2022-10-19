
#include "plansys2_pddl_parser/Domain.h"

namespace parser { namespace pddl {

        void Unknown::PDDLPrint( std::ostream & s, unsigned indent, const TokenStruct< std::string > & ts, const Domain & d ) const {
            tabindent( s, indent );
            s << "( unknown ";
            conds[0]->PDDLPrint( s, 0, ts, d );
            tabindent( s, indent );
            s << ")";
        }

        plansys2_msgs::msg::Node::SharedPtr Unknown::getTree( plansys2_msgs::msg::Tree & tree, const Domain & d, const std::vector<std::string> & replace ) const {
            throw UnsupportedConstruct("Unknown");
        }

        void Unknown::parse( Stringreader & f, TokenStruct< std::string > & ts, Domain & d ) {
            f.next();
            f.assert_token( "(" );
            Condition * condition = new TypeGround(d.preds.get( f.getToken( d.preds ) ) );
            condition->parse( f, ts, d );
            conds.push_back( condition );

            f.assert_token(")");
        }

    } } // namespaces
