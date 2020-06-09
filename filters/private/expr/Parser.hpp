#pragma once

#include <queue>
#include <stack>
#include <iostream>
#include <iomanip>

#include "Lexer.hpp"
#include "Expression.hpp"

namespace pdal
{
namespace expr
{

class Parser
{
public:
    Parser(Expression& expression) : m_expression(expression)
    {}

    bool parse(const std::string& s);
    std::string error() const
        { return m_error; }
    /**
    void prepare(PointLayoutPtr l);
    double eval(PointRef& p) const;
    **/

private:
    bool match(TokenType type);
    Token curToken() const;
    void setError(const std::string& err);

    bool expression();
    bool notexpr();
    bool orexpr();
    bool andexpr();
    bool compareexpr();
    bool addexpr();
    bool multexpr();
    bool uminus();
    bool primary();
    bool parexpr();

    Expression& m_expression;
    Lexer m_lexer;
    Token m_curTok;
    std::string m_error;
};

} // namespace expr
} // namespace pdal
