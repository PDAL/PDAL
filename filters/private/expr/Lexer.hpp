
#pragma once

#include <string>

#include "Token.hpp"

namespace pdal
{
namespace expr
{

class Lexer
{
public:
    Lexer() : m_pos(0)
    {}

    void lex(const std::string& s)
    {
        m_buf = s;
        m_pos = 0;
    }

    Token get(TokenClass cls);

private:
    Token get(char c);
    Token arithmeticOperator();
    Token logicalOperator();
    Token comparisonOperator();
    Token misc();
    Token number();
    Token dimension();

    std::string m_buf;
    std::string::size_type m_pos;
};

} // namespace expr
} // namespace pdal

