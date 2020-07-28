#include "AssignExpression.hpp"

namespace pdal
{
namespace expr
{

Expression& AssignExpression::valueExpr()
{
    return m_valueExpr;
}

Expression& AssignExpression::conditionalExpr()
{
    return m_conditionalExpr;
}

Utils::StatusWithReason AssignExpression::prepare(PointLayoutPtr layout)
{
    return m_valueExpr.prepare(layout) && m_conditionalExpr.prepare(layout);
}

} // namespace expr
} // namespace pdal

