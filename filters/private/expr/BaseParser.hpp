#pragma once

#include "Lexer.hpp"
#include "Expression.hpp"

namespace pdal
{
namespace expr
{

class BaseParser
{
public:
    BaseParser(Lexer& lexer) : m_lexer(lexer), m_endTokens { Token(TokenType::Eof) }
    {}

    BaseParser(Lexer& lexer, const Token& endToken) : m_lexer(lexer), m_endTokens { endToken }
    {}

    BaseParser(Lexer& lexer, const std::vector<Token>& endTokens) :
        m_lexer(lexer), m_endTokens(endTokens)
    {}

    bool checkEnd();
    std::string error() const
        { return m_error; }

protected:
    bool match(TokenType type);
    Token peekToken();
    Token curToken() const;
    Token lastToken() const;
    void setError(const std::string& err);
    void clearError();
    Lexer& lexer()
        { return m_lexer; }

private:
    Lexer& m_lexer;
    std::vector<Token> m_endTokens;
    Token m_curTok;
    std::string m_error;
};

} // namespace expr
} // namespace pdal
