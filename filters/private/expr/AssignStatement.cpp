#include "AssignStatement.hpp"

namespace pdal
{
namespace expr
{

MathExpression& AssignStatement::valueExpr()
{
    return m_valueExpr;
}

ConditionalExpression& AssignStatement::conditionalExpr()
{
    return m_conditionalExpr;
}

IdentExpression& AssignStatement::identExpr()
{
    return m_identExpr;
}

bool AssignStatement::valid() const
{
    return m_identExpr.valid();
}

AssignStatement::operator bool() const
{
    return valid();
}

std::string AssignStatement::print() const
{
    std::string s;
    s = "Ident = " + m_identExpr.print() + "\n";
    s += "Value = " + m_valueExpr.print() + "\n";
    s += "Condition = " + m_conditionalExpr.print() + "\n";
    return s;
}

Utils::StatusWithReason AssignStatement::prepare(PointLayoutPtr layout)
{
    auto status = m_identExpr.prepare(layout);
    if (!status)
        return {-1, m_identExpr.error()};

    status = m_valueExpr.prepare(layout);
    if (!status)
        return {-1, m_identExpr.error()};

    return m_conditionalExpr.prepare(layout);
}

} // namespace expr
} // namespace pdal

