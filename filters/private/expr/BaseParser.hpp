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

class BaseParser
{
public:
    Parser(Lexer& lexer) : m_lexer(lexer)
    {}

    bool parse(Expression& expr);
    std::string error() const
        { return m_error; }

protected:
    bool match(TokenType type);
    Token curToken() const;
    Token lastToken() const;
    void setError(const std::string& err);
    Lexer& lexer()
        { return m_lexer; }

private:
    Lexer& m_lexer;
    Token m_curTok;
    std::string m_error;
};

} // namespace expr
} // namespace pdal
