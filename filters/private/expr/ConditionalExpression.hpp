#pragma once

#include "Expression.hpp"
#include "Lexer.hpp"
#include "ConditionalParser.hpp"

namespace pdal
{
namespace expr
{

class ConditionalExpression : public Expression
{
public:
    Utils::StatusWithReason prepare(PointLayoutPtr layout);
    bool eval(PointRef& p) const;
};

} // namespace expr

namespace Utils
{

template<>
inline StatusWithReason fromString(const std::string& from,
    pdal::expr::ConditionalExpression& expr)
{
    expr::Lexer lexer(from);
    expr::ConditionalParser parser(lexer);
    bool ok = parser.expression(expr) && parser.checkEnd();
    return { ok ? 0 : -1, expr.error() };
}

} // namespace Util

} // namespace pdal
