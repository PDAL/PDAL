#pragma once

#include "AssignParser.hpp"
#include "ConditionalExpression.hpp"
#include "MathExpression.hpp"
#include "Lexer.hpp"

namespace pdal
{
namespace expr
{

class AssignExpression : public Expression
{
public:
    Expression& valueExpr();
    Expression& conditionalExpr();

    virtual Utils::StatusWithReason prepare(PointLayoutPtr layout);

private:
    MathExpression m_valueExpr;
    ConditionalExpression m_conditionalExpr;
};

} // namespace expr

namespace Utils
{

template<>
inline StatusWithReason fromString(const std::string& from,
    pdal::expr::AssignExpression& expr)
{
    expr::Lexer lexer(from);
    expr::AssignParser parser(lexer);
    bool ok = parser.expression(expr) && parser.checkEnd();
    return { ok ? 0 : -1, expr.error() };
}

} // namespace Util

} // namespace pdal
