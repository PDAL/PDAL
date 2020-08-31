#include "MathExpression.hpp"

namespace pdal
{
namespace expr
{

Utils::StatusWithReason MathExpression::prepare(PointLayoutPtr layout)
{
    Node *top = topNode();
    if (top)
    {
        auto status = top->prepare(layout);
        if (status)
        {
            if (!top->isValue())
                status = { -1, "Expression doesn't evaluate to a value." };
        }
        return status;
    }
    return true;
}

double MathExpression::eval(PointRef& p) const
{
    const Node *n = topNode();
    return n ? n->eval(p).m_dval : 0;
}

} // namespace expr
} // namespace pdal

