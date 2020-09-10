#include "ConditionalExpression.hpp"

namespace pdal
{
namespace expr
{

Utils::StatusWithReason ConditionalExpression::prepare(PointLayoutPtr layout)
{
    Node *top = topNode();
    if (top)
    {
        auto status = top->prepare(layout);
        if (status)
        {
            if (top->isValue())
                status =
                    { -1, "Expression evaluates to a value, not a boolean." };
            else
            {
                ConstLogicalNode *n = dynamic_cast<ConstLogicalNode *>(top);
                if (n)
                {
                    if (n->value())
                        status = { -1, "Expression is always true." };
                    else
                        status = { -1, "Expression is always false." };
                }
            }
        }
        return status;
    }
    return true;
}

bool ConditionalExpression::eval(PointRef& p) const
{
    const Node *n = topNode();
    return n ? n->eval(p).m_bval : true;
}

} // namespace expr
} // namespace pdal

