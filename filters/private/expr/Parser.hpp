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
    bool parse(const std::string& s, Expression& expr);
    std::string error() const
        { return m_error; }

private:
    bool match(TokenType type);
    Token curToken() const;
    Token lastToken() const;
    void setError(const std::string& err);

protected:
    virtual bool expression(Expression& expr);
    virtual bool notexpr(Expression& expr);
    virtual bool orexpr(Expression& expr);
    virtual bool andexpr(Expression& expr);
    virtual bool compareexpr(Expression& expr);
    virtual bool addexpr(Expression& expr);
    virtual bool multexpr(Expression& expr);
    virtual bool uminus(Expression& expr);
    virtual bool primary(Expression& expr);
    virtual bool parexpr(Expression& expr);

    Expression& m_expression;
    Lexer m_lexer;
    Token m_curTok;
    std::string m_error;
};

} // namespace expr
} // namespace pdal
