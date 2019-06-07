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

#pragma once

#include <string>

namespace pdal
{
namespace expr
{

enum class TokenClass
{
    Operator,
    Any
};

enum class TokenType
{
    Eof,
    Error,
    Plus,
    Minus,
    Divide,
    Multiply,
    Equal,
    Inequality,
    Greater,
    GreaterOrEqual,
    Less,
    LessOrEqual,
    In,
    And,
    Or,
    Lparen,
    Rparen,
    Comma,
    Number,
    Dimension
};

class Token
{
    // Union with string requires a bunch of muck, so...
    struct Value
    {
        double d;
        std::string s;
    };

public:
    Token(TokenType type, std::string::size_type start,
            std::string::size_type end, double d) :
        m_type(type), m_start(start), m_end(end)
    { m_val.d = d; }

    Token(TokenType type, std::string::size_type start,
            std::string::size_type end, const std::string& s) :
        m_type(type), m_start(start), m_end(end)
    { m_val.s = s; }

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

private:
    TokenType m_type;
    std::string::size_type m_start;
    std::string::size_type m_end;
    Value m_val;
};

} // namespace expr
} // namespace pdal
