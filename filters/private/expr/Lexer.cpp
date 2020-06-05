
#include "Lexer.hpp"

namespace pdal
{
namespace expr
{

Token Lexer::get(TokenClass cls)
{
    char c;

    size_t originalPos = m_pos;
    while (isspace(m_buf[m_pos]))
        m_pos++;

    if (m_pos == m_buf.size())
        return Token(TokenType::Eof);

    Token tok;
    if (cls == TokenClass::Or)
    {
        tok = logicalOperator();
        if (tok.type() != TokenType::Or)
            tok = Token(TokenType::Error);
    }
    else if (cls == TokenClass::And)
    {
        tok = logicalOperator();
        if (tok.type() != TokenType::And)
            tok = Token(TokenType::Error);
    }
    else if (cls == TokenClass::Compare)
        tok = comparisonOperator();
    else if (cls == TokenClass::Add)
    {
        tok = arithmeticOperator();
        if (tok.type() != TokenType::Add && tok.type() != TokenType::Subtract)
            tok = Token(TokenType::Error);
    }
    else if (cls == TokenClass::Multiply)
    {
        tok = arithmeticOperator();
        if (tok.type() != TokenType::Multiply &&
              tok.type() != TokenType::Divide)
            tok = Token(TokenType::Error);
    }
    else if (cls == TokenClass::Primary)
    {
        tok = dimension();
        if (!tok)
        {
            tok = number();
            if (!tok)
                tok = Token(TokenType::Error);
        }
    }
    else if (cls == TokenClass::Lparen)
    {
        tok = misc();
        if (tok.type() != TokenType::Lparen)
            tok = Token(TokenType::Error);
    }
    else if (cls == TokenClass::Rparen)
    {
        tok = misc();
        if (tok.type() != TokenType::Rparen)
            tok = Token(TokenType::Error);
    }

    if (tok)
        m_pos = tok.end();
    else
        m_pos = originalPos;
    return tok;
}


Token Lexer::misc()
{
    char c = m_buf[m_pos];
    if (c == '(')
        return Token(TokenType::Lparen, m_pos, m_pos + 1);
    if (c == ')')
        return Token(TokenType::Rparen, m_pos, m_pos + 1);
    return Token(TokenType::Error);
}

Token Lexer::arithmeticOperator()
{
    char c = m_buf[m_pos];

    //ABELL - Need to check for rparen?
    if (c == '+')
    {
        char d = m_buf[m_pos + 1];
        if (d != '+')
            return Token(TokenType::Add, m_pos, m_pos + 1);
    }
    if (c == '-')
    {
        char d = m_buf[m_pos + 1];
        if (d != '-')
            return Token(TokenType::Subtract, m_pos, m_pos + 1);
    }
    if (c == '/')
        return Token(TokenType::Divide, m_pos, m_pos + 1);
    if (c == '*')
        return Token(TokenType::Multiply, m_pos, m_pos + 1);
    return Token(TokenType::Error);
}

Token Lexer::comparisonOperator()
{
    Token tok(TokenType::Error);

    char c = m_buf[m_pos];
    if (!c)
        return Token(TokenType::Error);

    char d = m_buf[m_pos + 1];
    if (d)
    {
        if (c == '=' && d == '=')
            return Token(TokenType::Equal, m_pos, m_pos + 2);
        if (c == '!' && d == '=')
            return Token(TokenType::NotEqual, m_pos, m_pos + 2);
        if (c == '>' && d == '=')
            return Token(TokenType::GreaterEqual, m_pos, m_pos + 2);
        if (c == '<' && d == '=')
            return Token(TokenType::LessEqual, m_pos, m_pos + 2);
    }
    if (c == '>')
        return Token(TokenType::Greater, m_pos, m_pos + 1);
    if (c == '<')
        return Token(TokenType::Less, m_pos, m_pos + 1);
    return Token(TokenType::Error);
}

Token Lexer::logicalOperator()
{
    char c = m_buf[m_pos];
    if (!c)
        return Token(TokenType::Error);
    char d = m_buf[m_pos + 1];
    if (d)
    {
        if (c == '&' && d == '&')
            return Token(TokenType::And, m_pos, m_pos + 2);
        if (c == '|' && d == '|')
            return Token(TokenType::Or, m_pos, m_pos + 2);
    }
    return Token(TokenType::Error);
}


Token Lexer::number()
{
    const char *start = m_buf.data() + m_pos;
    char *end;

    double v = strtod(start, &end);
    if (start == end)
        return Token(TokenType::Error);
    return Token(TokenType::Number, m_pos, end - m_buf.data(), v);
}


Token Lexer::dimension()
{
    size_t end;
    for (end = m_pos; end < m_buf.size(); ++end)
        if (!std::isalpha((int)m_buf[end]))
            break;

    if (end > m_pos)
        return Token(TokenType::Dimension, m_pos, end,
            m_buf.substr(m_pos, end - m_pos));
    return Token(TokenType::Error);
     
/**    
    size_t end = Dimension::extractName(m_buf, m_pos);
    return Token(TokenType::Dimension, m_pos, m_pos + end,
        m_buf.substr(m_pos, end));
**/
}

} // namespace expr
} // namespace pdal
