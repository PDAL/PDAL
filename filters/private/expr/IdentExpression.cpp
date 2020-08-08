#include "IdentExpression.hpp"

namespace pdal
{
namespace expr
{

Utils::StatusWithReason IdentExpression::prepare(PointLayoutPtr layout)
{
    Node *top = topNode();
    if (top)
        return top->prepare(layout);
    return false;
}

Dimension::Id IdentExpression::eval() const
{
    const VarNode *n = dynamic_cast<const VarNode *>(topNode());
    return n ? n->eval() : Dimension::Id::Unknown;
}

} // namespace expr
} // namespace pdal

