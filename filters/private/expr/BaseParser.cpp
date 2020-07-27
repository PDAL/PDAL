#include "Parser.hpp"

namespace pdal
{
namespace expr
{

bool Parser::parse(Expression& expr)
{
    m_lexer.reset(s);
    m_error.clear();

    expr.clear();
    bool ok = expression(expr);
    if (ok)
    {
        // If we're at the end, we should have exhausted all tokens.
        Token tok = m_lexer.get();
        if (tok != TokenType::Eof)
        {
            m_error = "Found '" + tok.sval() + "' after valid expression.";
            return false;
        }
    }
    return ok;
}

Token Parser::curToken() const
{
    return m_curTok;
}

bool Parser::match(TokenType type)
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

void Parser::setError(const std::string& err)
{
    if (m_error.empty())
        m_error = err;
}

} // namespace expr
} // namespace pdal
