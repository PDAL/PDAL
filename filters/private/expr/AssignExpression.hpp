#pragma once

#include <stack>
#include <string>
#include <memory>

#include <pdal/Dimension.hpp>
#include <pdal/PointLayout.hpp>
#include <pdal/PointRef.hpp>
#include <pdal/util/Utils.hpp>

namespace pdal
{
namespace expr
{

class AssignExpression : public Expression
{
public:
    AssignExpression();
    AssignExpression(const Expression& expr);
    AssignExpression& operator=(const Expression& expr);
    ~AssignExpression();

    bool parse(const std::string& s);
    Utils::StatusWithReason prepare(PointLayoutPtr layout);
    Expression& valueExpr();
    Expression& conditionalExpr();

private:
    std::string m_error;
    Expression m_valueExpr;
    Expression m_conditionalExpr;

    friend std::ostream& operator<<(std::ostream& out, const AssignExpression& expr);
};

} // namespace expr

namespace Utils
{

template<>
inline StatusWithReason fromString(const std::string& from,
    pdal::expr::Expression& expr)
{
    bool ok = expr.parse(from);
    return { ok ? 0 : -1, expr.error() };
}

} // namespace Util

} // namespace pdal
