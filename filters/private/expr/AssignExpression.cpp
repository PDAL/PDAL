#include "AssignExpression.hpp"

namespace pdal
{
namespace expr
{

AssignExpression::valueExpr()
{
    return m_valueExpr;
}

AssignExpression::conditionalExpr()
{
    return m_conditionalExpr;
}

Utils::StatusWithReason AssignExpression::prepare(PointLayoutPtr layout)
{
    return m_valueExpr.prepare() && m_conditionalExpr().prepare();
}

bool Expression::eval(PointRef& p) const
{
    return m_nodes.top()->eval(p).m_bval;
}

} // namespace expr
} // namespace pdal

