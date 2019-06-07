/******************************************************************************
* Copyright (c) 2019, Andrew Bell (andrew.bell.ia@gmail.com)
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
*     * The name of Andrew Bell may not be used to endorse or promote
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

#include "Lexer.hpp"
#include <pdal/DimUtil.hpp>

namespace pdal
{
namespace expr
{

Token Lexer::get(TokenClass cls)
{
    while (isspace(m_buf[m_pos]))
        m_pos++;

    if (m_pos == m_buf.size())
        return Token(TokenType::Eof);

    Token tok;
    if (cls == TokenClass::Operator)
        tok = getOperator(m_buf[m_pos]);
    else
        tok = get(m_buf[m_pos]);
    m_pos = tok.end();
    return tok;
}


Token Lexer::get(char c)
{
    Token tok(TokenType::Error);

    tok = getOperator(c);
    if (tok.valid())
        return tok;

    if (c == '+' || c == '-' || isdigit(c))
        tok = number();
    else if (c == '(')
        tok = Token(TokenType::Lparen, m_pos, m_pos + 1);
    else if (c == ')')
        tok = Token(TokenType::Rparen, m_pos, m_pos + 1);
    else if (isalpha(c))
        tok = dimension();

    return tok;
}


Token Lexer::getOperator(char c)
{
    Token tok(TokenType::Error);

    if (c == ')')
        tok = Token(TokenType::Rparen, m_pos, m_pos + 1);
    else if (c == ',')
        tok = Token(TokenType::Comma, m_pos, m_pos + 1);
    else if (c == '+')
        tok = Token(TokenType::Plus, m_pos, m_pos + 1);
    else if (c == '-')
        tok = Token(TokenType::Minus, m_pos, m_pos + 1);
    else if (c == '/')
        tok = Token(TokenType::Divide, m_pos, m_pos + 1);
    else if (c == '*')
        tok = Token(TokenType::Multiply, m_pos, m_pos + 1);
    else if (m_buf.size() > m_pos + 1)
    {
        char d = m_buf[m_pos + 1];

        if (c == '>' && d == '=') 
            tok = Token(TokenType::GreaterOrEqual, m_pos, m_pos + 2);
        else if (c == '<' && d == '=') 
            tok = Token(TokenType::LessOrEqual, m_pos, m_pos + 2);
        else if (c == '=' && d == '=') 
            tok = Token(TokenType::Equal, m_pos, m_pos + 2);
        else if (c == '<' && d == '>') 
            tok = Token(TokenType::Inequality, m_pos, m_pos + 2);
        else if (c == '!' && d == '=') 
            tok = Token(TokenType::Inequality, m_pos, m_pos + 2);
        else if (c == '>') 
            tok = Token(TokenType::Greater, m_pos, m_pos + 1);
        else if (c == '<') 
            tok = Token(TokenType::Less, m_pos, m_pos + 1);
        else if (c == '&' && d == '&')
            tok = Token(TokenType::And, m_pos, m_pos + 2);
        else if (c == '|' && d == '|')
            tok = Token(TokenType::Or, m_pos, m_pos + 2);
        else if (m_buf.size() > m_pos + 2 
            && std::toupper(c) == 'I' 
            && std::toupper(d) == 'N')
        {
            size_t end_pos = m_pos + 2;

            while (isspace(m_buf[end_pos]))
                end_pos++;

            if (end_pos < m_buf.size() && m_buf[end_pos] == '(')
                tok = Token(TokenType::In, m_pos, end_pos);
        }
    }
    return tok;
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
    size_t end = Dimension::extractName(m_buf, m_pos);
    return Token(TokenType::Dimension, m_pos, m_pos + end,
        m_buf.substr(m_pos, end));
}

} // namespace expr
} // namespace pdal
