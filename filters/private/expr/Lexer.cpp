
#include <cassert>
#include <cctype>
#include <cstdlib>

#include "Lexer.hpp"

namespace pdal
{
namespace expr
{

namespace
{
struct EofException
{};

}

char Lexer::getChar()
{
    char c = 0;
    if (m_pos < m_buf.size())
        c = m_buf[m_pos];
    m_pos++;
    return c;
}

void Lexer::putChar()
{
    assert(m_pos != 0);

    m_pos--;
}

Token Lexer::get()
{
    char c;
    Token tok;

    if (m_pos >= m_buf.size())
        return Token(TokenType::Eof, m_buf.size(), m_buf.size(), "");
    while (true)
    {
        m_tokPos = m_pos;
        c = getChar();

        if (std::isspace(c))
            continue;

        tok = top(c);
        break;
    }
    return tok;
}

void Lexer::put(Token t)
{
    m_pos = t.m_start;
}

void Lexer::putEnd(Token t)
{
    m_pos = t.m_end;
}

Token Lexer::top(char c)
{
    Token tok;

    switch (c)
    {
    case '&':
        tok = ampersand();
        break;
    case '|':
        tok = bar();
        break;
    case '!':
        tok = exclamation();
        break;
    case '-':
        tok = dash();
        break;
    case '<':
        tok = less();
        break;
    case '>':
        tok = greater();
        break;
    case '=':
        tok = equal();
        break;
    case '+':
        tok = Token(TokenType::Plus, m_tokPos, m_pos, "+");
        break;
    case '*':
        tok = Token(TokenType::Asterisk, m_tokPos, m_pos, "*");
        break;
    case '/':
        tok = Token(TokenType::Slash, m_tokPos, m_pos, "/");
        break;
    case '(':
        tok = Token(TokenType::Lparen, m_tokPos, m_pos, "(");
        break;
    case ')':
        tok = Token(TokenType::Rparen, m_tokPos, m_pos, ")");
        break;
    case '0': case '1': case '2': case '3': case '4':
    case '5': case '6': case '7': case '8': case '9':
        tok = number();
        break;
    case 'a': case 'b': case 'c': case 'd': case 'e':
    case 'f': case 'g': case 'h': case 'i': case 'j':
    case 'k': case 'l': case 'm': case 'n': case 'o':
    case 'p': case 'q': case 'r': case 's': case 't':
    case 'u': case 'v': case 'w': case 'x': case 'y':
    case 'z':
    case 'A': case 'B': case 'C': case 'D': case 'E':
    case 'F': case 'G': case 'H': case 'I': case 'J':
    case 'K': case 'L': case 'M': case 'N': case 'O':
    case 'P': case 'Q': case 'R': case 'S': case 'T':
    case 'U': case 'V': case 'W': case 'X': case 'Y':
    case 'Z':
        tok = letter();
        break;
    default:
        tok = Token(TokenType::Error, m_tokPos, m_pos, "Syntax error.");    
        break;
    }
    return tok;
}

Token Lexer::ampersand()
{
    char c = getChar();
    if (c == '&')
        return Token(TokenType::And, m_tokPos, m_pos, "&&");
    putChar();
    return Token(TokenType::Error, m_tokPos, m_pos, "'&' invalid in this "
        "context.");
}

Token Lexer::bar()
{
    char c = getChar();
    if (c == '|')
        return Token(TokenType::Or, m_tokPos, m_pos, "||");
    putChar();
    return Token(TokenType::Error, m_tokPos, m_pos, "'!' invalid in this "
        "context.");
}

Token Lexer::exclamation()
{
    char c = getChar();
    if (c == '=')
        return Token(TokenType::NotEqual, m_tokPos, m_pos, "!=");
    putChar();
    return Token(TokenType::Not, m_tokPos, m_pos, "!");
}

Token Lexer::dash()
{
    char c = getChar();
    putChar();
    if (c != '-')
        return Token(TokenType::Dash, m_tokPos, m_pos, "-");
    return Token(TokenType::Error, m_tokPos, m_pos,
        "Found disallowed consecutive dashes: '--'");
}

Token Lexer::equal()
{
    char c = getChar();
    if (c == '=')
        return Token(TokenType::Equal, m_tokPos, m_pos, "==");
    putChar();
    return Token(TokenType::Assign, m_tokPos, m_pos, "=");
}

Token Lexer::less()
{
    char c = getChar();
    if (c == '=')
        return Token(TokenType::LessEqual, m_tokPos, m_pos, "<=");
    putChar();
    return Token(TokenType::Less, m_tokPos, m_pos, "<");
}

Token Lexer::greater()
{
    char c = getChar();
    if (c == '=')
        return Token(TokenType::GreaterEqual, m_tokPos, m_pos, ">=");
    putChar();
    return Token(TokenType::Greater, m_tokPos, m_pos, ">");
}

Token Lexer::number()
{
    const char *start = m_buf.data() + m_tokPos;
    char *end;
    double v = strtod(start, &end);
    m_pos = end - m_buf.data();
    return Token(TokenType::Number, m_tokPos, m_pos,
        m_buf.substr(m_tokPos, m_pos), v);
}

Token Lexer::letter()
{
    while (true)
    {
        char c = getChar();
        if (!std::isalnum(c) && c != '_')
        {
            putChar();
            return Token(TokenType::Identifier, m_tokPos, m_pos, 
                m_buf.substr(m_tokPos, m_pos - m_tokPos));
        }
    } 
}

} // namespace expr
} // namespace pdal
