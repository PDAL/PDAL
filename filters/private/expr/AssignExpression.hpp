#pragma once

#include "AssignParser.hpp"
#include "ConditionalExpression.hpp"
#include "IdentExpression.hpp"
#include "MathExpression.hpp"
#include "Lexer.hpp"

namespace pdal
{
namespace expr
{

class AssignExpression : public Expression
{
public:
    AssignExpression() = default;
    AssignExpression(const AssignExpression&) = default;
    AssignExpression(AssignExpression&&) = default;
    AssignExpression& operator=(const AssignExpression&) = default;

    virtual ~AssignExpression()
    {}

    MathExpression& valueExpr();
    IdentExpression& identExpr();
    ConditionalExpression& conditionalExpr();
    bool valid() const;
    operator bool() const;

    std::string print() const;
    Utils::StatusWithReason prepare(PointLayoutPtr layout);

private:
    IdentExpression m_identExpr;
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
