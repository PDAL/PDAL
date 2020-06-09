
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
    Lexer() : m_pos(0), m_tokPos(0)
    {}
    Lexer(const std::string& s) : m_buf(s), m_pos(0), m_tokPos(0)
    {}

    void reset(const std::string& s)
    {
        m_buf = s;
        m_pos = 0;
        m_tokPos = 0;
    }
    Token get();
    void put(Token t);
    void putEnd(Token t);

private:
    char getChar();
    void putChar();
    Token top(char c);
    Token ampersand();
    Token bar();
    Token exclamation();
    Token dash();
    Token equal();
    Token less();
    Token greater();
    Token number();
    Token letter();

    std::string m_buf;
    std::string::size_type m_pos;
    std::string::size_type m_tokPos;
};

} // namespace expr
} // namespace pdal

