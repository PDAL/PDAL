
#pragma once

#include <string>

namespace pdal
{
namespace expr
{

enum class TokenType
{
    Eof,
    Error,

    Assign,

    Plus,
    Dash,
    Slash,
    Asterisk,

    Lparen,
    Rparen,

    Not,
    Or,
    And,
    Greater,
    Less,
    Equal,
    NotEqual,
    LessEqual,
    GreaterEqual,

    Number,
    Identifier
};

class Token
{
    friend class Lexer;

    // Union with string requires a bunch of muck, so...
    struct Value
    {
        double d;
        std::string s;
    };

public:
    Token(TokenType type, std::string::size_type start,
            std::string::size_type end, const std::string& s, double d = 0) :
        m_type(type), m_start(start), m_end(end)
    {
        m_val.s = s;
        m_val.d = d;
    }


    Token(TokenType type, std::string::size_type start,
            std::string::size_type end) :
        m_type(type), m_start(start), m_end(end)
    {}
    Token(TokenType type) : m_type(type), m_start(0), m_end(0)
    {}
    Token() : m_type(TokenType::Error), m_start(0), m_end(0)
    {}

    TokenType type() const
    { return m_type; }

    std::string::size_type start() const
    { return m_start; }

    std::string::size_type end() const
    { return m_end; }

    bool valid() const
    { return m_type != TokenType::Error; }

    double dval() const
    { return m_val.d; }

    std::string sval() const
    { return m_val.s; }

    operator bool () const
    { return valid() && m_type != TokenType::Eof; }

    bool operator == (TokenType type) const
    { return m_type == type; }

    bool operator == (const Token& other) const
    { return m_type == other.m_type; }

    bool operator != (TokenType type) const
    { return m_type != type; }

    bool operator != (const Token& other) const
    { return m_type != other.m_type; }

private:
    TokenType m_type;
    std::string::size_type m_start;
    std::string::size_type m_end;
    Value m_val;
};

} // namespace expr
} // namespace pdal
