#include "AssignExpression.hpp"

namespace pdal
{
namespace expr
{

/**
AssignExpression::~AssignExpression()
{}
**/

MathExpression& AssignExpression::valueExpr()
{
    return m_valueExpr;
}

ConditionalExpression& AssignExpression::conditionalExpr()
{
    return m_conditionalExpr;
}

IdentExpression& AssignExpression::identExpr()
{
    return m_identExpr;
}

bool AssignExpression::valid() const
{
    return m_identExpr.valid();
}

AssignExpression::operator bool() const
{
    return valid();
}

std::string AssignExpression::print() const
{
    std::string s;
    s = "Ident = " + m_identExpr.print() + "\n";
    s += "Value = " + m_valueExpr.print() + "\n";
    s += "Condition = " + m_conditionalExpr.print() + "\n";
    return s;
}

Utils::StatusWithReason AssignExpression::prepare(PointLayoutPtr layout)
{
    return m_identExpr.prepare(layout) &&
        m_valueExpr.prepare(layout) && m_conditionalExpr.prepare(layout);
}

} // namespace expr
} // namespace pdal

