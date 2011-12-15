/******************************************************************************
* Copyright (c) 2011, Michael P. Gerlek (mpg@flaxen.com)
*
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following
* conditions are met:
*
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in
*       the documentation and/or other materials provided
*       with the distribution.
*     * Neither the name of Hobu, Inc. or Flaxen Geo Consulting nor the
*       names of its contributors may be used to endorse or promote
*       products derived from this software without specific prior
*       written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
* OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
* AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
* OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
* OF SUCH DAMAGE.
****************************************************************************/

#include <pdal/plang/Parser.hpp>

#include <pdal/plang/AstNode.hpp>
#include <pdal/plang/Program.hpp>
#include <pdal/plang/SymbolTable.hpp>

#ifdef PDAL_COMPILER_MSVC
#  pragma warning(push)
#  pragma warning(disable: 4512)  // assignment operator could not be generated
#  pragma warning(disable: 4127)  // conditional expression is constant
#  pragma warning(disable: 4100)  // unreferenced formal
#endif

#include <boost/spirit/include/qi.hpp>
#include <boost/spirit/include/phoenix_operator.hpp>
#include <boost/spirit/include/phoenix_object.hpp>

#ifdef PDAL_COMPILER_MSVC
#  pragma warning(pop)
#endif


#include <iostream>
#include <string>
#include <complex>
#include <vector>


namespace qi = boost::spirit::qi;
namespace pho = boost::phoenix;
namespace ascii = boost::spirit::ascii;


namespace pdal { namespace plang {


struct PlangParser : qi::grammar<std::string::const_iterator, Program*(), ascii::space_type>
{
    PlangParser();

    qi::rule<std::string::const_iterator, Program*(), ascii::space_type> program;
    qi::rule<std::string::const_iterator, AstNode*(), ascii::space_type> assignments;
    qi::rule<std::string::const_iterator, AstNode*(), ascii::space_type> assignment;
    qi::rule<std::string::const_iterator, SymbolTable*(), ascii::space_type> declarations;
    qi::rule<std::string::const_iterator, Symbol*(), ascii::space_type> declaration;
    qi::rule<std::string::const_iterator, AstNode*(), ascii::space_type> lhs;
    qi::rule<std::string::const_iterator, AstNode*(), ascii::space_type> rhs;
    qi::rule<std::string::const_iterator, AstNode*(), ascii::space_type> constant;
    qi::rule<std::string::const_iterator, SymbolName*(), ascii::space_type> variable;
    qi::rule<std::string::const_iterator, AstNode*(), ascii::space_type> predicate;
    qi::rule<std::string::const_iterator, AstNode*(), ascii::space_type> predicate_tail;
    qi::rule<std::string::const_iterator, AstNode*(), ascii::space_type> predicate_tail_star;
    qi::rule<std::string::const_iterator, AstNode*(), ascii::space_type> expr;
    qi::rule<std::string::const_iterator, AstNode*(), ascii::space_type> expr_tail;
    qi::rule<std::string::const_iterator, AstNode*(), ascii::space_type> expr_tail_star;
    qi::rule<std::string::const_iterator, AstNode*(), ascii::space_type> term;
    qi::rule<std::string::const_iterator, AstNode*(), ascii::space_type> term_tail;
    qi::rule<std::string::const_iterator, AstNode*(), ascii::space_type> term_tail_star;
    qi::rule<std::string::const_iterator, AstNode*(), ascii::space_type> factor;
};


PlangParser::PlangParser() : PlangParser::base_type(program)
{
#define SET_P(T, p)      qi::_val = pho::new_<T>(p)
#define SET_1(T)         qi::_val = pho::new_<T>(qi::_1)
#define SET_1_P(T, p)    qi::_val = pho::new_<T>(qi::_1, p)
#define SET_1_2(T)       qi::_val = pho::new_<T>(qi::_1, qi::_2)
#define SET_1_2_P(T, p)  qi::_val = pho::new_<T>(qi::_1, qi::_2, p)
#define PASS             qi::_val = qi::_1

    qi::real_parser<float, qi::strict_real_policies<float> > strict_float;
    qi::real_parser<double, qi::strict_real_policies<double> > strict_double;

    constant = ( ("0x" >> qi::hex >> 'l')       [ SET_1(AstConstant) ]
               | ("0x" >> qi::hex)              [ SET_1(AstConstant) ]
               | (strict_float >> 'f')          [ SET_1(AstConstant) ]
               | strict_double                  [ SET_1(AstConstant) ]
               | (qi::ulong_long >> "ul")       [ SET_1(AstConstant) ]
               | (qi::ulong_long >> "lu")       [ SET_1(AstConstant) ]
               | (qi::long_long >> 'l')         [ SET_1(AstConstant) ]
               | (qi::uint_ >> 'u')             [ SET_1(AstConstant) ]
               | qi::int_                       [ SET_1(AstConstant) ]
               | qi::lit("true")                [ SET_P(AstConstant, true) ]
               | qi::lit("false")               [ SET_P(AstConstant, false) ]
               )
             ;

    // (letter or _) followed by *(letter or number or _)
    variable = ( (qi::alpha|qi::char_('_'))  >> *(qi::alpha | qi::digit | qi::char_('_')))    [ SET_1_2(SymbolName) ]
             ;

    predicate_tail = ( ">" >> expr     [ SET_1_P(AstTempOp1, NodeType_Greater1) ]
                     | ">=" >> expr    [ SET_1_P(AstTempOp1, NodeType_GreaterEq1) ]
                     | "<" >> expr     [ SET_1_P(AstTempOp1, NodeType_Less1) ]
                     | "<=" >> expr    [ SET_1_P(AstTempOp1, NodeType_LessEq1) ]
                     | "==" >> expr    [ SET_1_P(AstTempOp1, NodeType_Equal1) ]
                     | "!=" >> expr    [ SET_1_P(AstTempOp1, NodeType_NotEqual1) ]
                     | "&&" >> expr    [ SET_1_P(AstTempOp1, NodeType_LogicalAnd1) ]
                     | "||" >> expr    [ SET_1_P(AstTempOp1, NodeType_LogicalOr1) ]
                     ) 
                   ;

    predicate_tail_star = (*predicate_tail)    [ SET_1(AstTempVector) ]
                        ;
                  
    predicate = (expr >> predicate_tail_star)    [ SET_1_2(AstTempVector) ]
              ;
         
    expr_tail = ( '+' >> term    [ SET_1_P(AstTempOp1, NodeType_Add1) ]
                | '-' >> term    [ SET_1_P(AstTempOp1, NodeType_Subtract1) ]
                | '^' >> term    [ SET_1_P(AstTempOp1, NodeType_ArithXor1) ]
                | '|' >> term    [ SET_1_P(AstTempOp1, NodeType_ArithOr1) ]
                | '&' >> term    [ SET_1_P(AstTempOp1, NodeType_ArithAnd1) ]
                ) 
              ;

    expr_tail_star = (*expr_tail)    [ SET_1(AstTempVector) ]
                   ;
                  
    expr = (term >> expr_tail_star)    [ SET_1_2(AstTempVector) ]
         ;

    term_tail = ( '*' >> factor    [ SET_1_P(AstTempOp1, NodeType_Multiply1) ]
                | '/' >> factor    [ SET_1_P(AstTempOp1, NodeType_Divide1) ]
                )
              ;

    term_tail_star = (*term_tail)    [ SET_1(AstTempVector) ]
                   ;

    term = (factor >> term_tail_star)    [ SET_1_2(AstTempVector) ]
         ;

    factor = ( constant                       [ PASS ]
             | ("uint8(" >> expr >> ')' )     [ SET_1_P(AstConvert, DataType_Uint8) ]
             | ("uint16(" >> expr >> ')' )    [ SET_1_P(AstConvert, DataType_Uint16) ]
             | ("uint32(" >> expr >> ')' )    [ SET_1_P(AstConvert, DataType_Uint32) ]
             | ("uint64(" >> expr >> ')' )    [ SET_1_P(AstConvert, DataType_Uint64) ]
             | ("int8(" >> expr >> ')' )      [ SET_1_P(AstConvert, DataType_Int8) ]
             | ("int16(" >> expr >> ')' )     [ SET_1_P(AstConvert, DataType_Int16) ]
             | ("int32(" >> expr >> ')' )     [ SET_1_P(AstConvert, DataType_Int32) ]
             | ("int64(" >> expr >> ')' )     [ SET_1_P(AstConvert, DataType_Int64) ]
             | ("float32(" >> expr >> ')' )   [ SET_1_P(AstConvert, DataType_Float32) ]
             | ("float64(" >> expr >> ')' )   [ SET_1_P(AstConvert, DataType_Float64) ]
             | variable                       [ SET_1(AstVariableUse) ]
             | ('(' >> expr >> ')')           [ PASS ]
             | ('-' >> factor)                [ SET_1(AstNegate) ]
             | ('+' >> factor)                [ PASS ]
             )                               
           ;

    rhs = predicate    [ PASS ]
        ;

    assignment = (variable >> '=' >> rhs )     [ SET_1_2(AstVariableDef) ]
               ;             

    declaration = ( ("uint8" >> variable)      [ SET_1_P(Symbol, DataType_Uint8) ]
                  | ("uint16" >> variable)     [ SET_1_P(Symbol, DataType_Uint16) ]
                  | ("uint32" >> variable)     [ SET_1_P(Symbol, DataType_Uint32) ]
                  | ("uint64" >> variable)     [ SET_1_P(Symbol, DataType_Uint64) ]
                  | ("int8" >> variable)       [ SET_1_P(Symbol, DataType_Int8) ]
                  | ("int16" >> variable)      [ SET_1_P(Symbol, DataType_Int16) ]
                  | ("int32" >> variable)      [ SET_1_P(Symbol, DataType_Int32) ]
                  | ("int64" >> variable)      [ SET_1_P(Symbol, DataType_Int64) ]
                  | ("float32" >> variable)    [ SET_1_P(Symbol, DataType_Float32) ]
                  | ("float64" >> variable)    [ SET_1_P(Symbol, DataType_Float64) ]
                  | ("bool" >> variable)       [ SET_1_P(Symbol, DataType_Bool) ]
                  )
                ;

    declarations = (*(declaration >> ';'))     [ SET_1(SymbolTable) ]
            ;

    assignments = (*(assignment >> ';'))     [ SET_1(AstTempVector) ]
            ;

    program = (declarations >> assignments)     [ SET_1_2(Program) ]
            ;

    return;
}


bool Parser::parse()
{
    std::string::const_iterator iter = m_text.begin();
    std::string::const_iterator end = m_text.end();

    using boost::spirit::ascii::space;
    PlangParser parser;

    bool ok = qi::phrase_parse(iter, end, parser, space, m_program);

    m_parsed = (ok && iter==end);

    return m_parsed;
}


} } // namespaces
