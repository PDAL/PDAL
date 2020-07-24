#pragma once

namespace pdal
{
namespace expr
{

class AssignParser : public Parser
{
public:
    bool parse(const std::string& s, AssignExpression& expr);

private:
    bool assignment(AssignExpression& expr);
    bool where(AssignExpression& expr);

    Token lastToken() const;
    void setError(const std::string& err);

protected:
    virtual bool valueexpr(Expression& expr);
    virtual bool multexpr(Expression& expr);
    virtual bool partexpr(Expression& expr);

    Lexer m_lexer;
    Token m_curTok;
    std::string m_error;
};

} // namespace expr
} // namespace pdal
