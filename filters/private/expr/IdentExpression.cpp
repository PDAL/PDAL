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


std::string IdentExpression::name() const
{
    const VarNode *n = dynamic_cast<const VarNode *>(topNode());

    n->name();
    if (n)
        return n->name();
    else
        return std::string("");

}


Dimension::Id IdentExpression::eval() const
{
    const VarNode *n = dynamic_cast<const VarNode *>(topNode());
    return n ? n->eval() : Dimension::Id::Unknown;
}

} // namespace expr
} // namespace pdal

