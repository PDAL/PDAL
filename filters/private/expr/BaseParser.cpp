#include "BaseParser.hpp"

namespace pdal
{
namespace expr
{

bool BaseParser::checkEnd()
{
    // If we're at the end, we should have exhausted all tokens.
    Token tok = m_lexer.get();
    if (tok != TokenType::Eof)
    {
        m_error = "Found '" + tok.sval() + "' after valid expression.";
        return false;
    }
    return true;
}

Token BaseParser::peekToken()
{
    Token t = m_lexer.get();
    m_lexer.put(t);
    return t;
}

Token BaseParser::curToken() const
{
    return m_curTok;
}

bool BaseParser::match(TokenType type)
{
    Token t = m_lexer.get();
    if (t.type() == type)
    {
        m_curTok = t;
        return true;
    }
    m_lexer.put(t);
    return false;
}

void BaseParser::setError(const std::string& err)
{
    if (m_error.empty())
        m_error = err;
}

void BaseParser::clearError()
{
    m_error.clear();
}

} // namespace expr
} // namespace pdal
