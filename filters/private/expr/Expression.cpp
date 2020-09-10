#include "Expression.hpp"

namespace pdal
{
namespace expr
{

Node::Node(NodeType type) : m_type(type)
{}

Node::~Node()
{}

NodeType Node::type() const
{ return m_type; }


//
// NotNode
//
NotNode::NotNode(NodeType type, NodePtr sub) :
    LogicalNode(type), m_sub(std::move(sub))
{}

std::string NotNode::print() const
{
    return "!(" + m_sub->print() + ")";
}

Utils::StatusWithReason NotNode::prepare(PointLayoutPtr l)
{
    return m_sub->prepare(l);
}

Result NotNode::eval(PointRef& p) const
{
    return !(m_sub->eval(p).m_bval);
}


//
// UnMathNode
//
UnMathNode::UnMathNode(NodeType type, NodePtr sub) :
    ValueNode(type), m_sub(std::move(sub))
{}

std::string UnMathNode::print() const
{
    return "-(" + m_sub->print() + ")";
}

Utils::StatusWithReason UnMathNode::prepare(PointLayoutPtr l)
{
    return m_sub->prepare(l);
}

Result UnMathNode::eval(PointRef& p) const
{
    return -(m_sub->eval(p).m_dval);
}


//
// BinMathNode
//
BinMathNode::BinMathNode(NodeType type, NodePtr left, NodePtr right) :
    ValueNode(type), m_left(std::move(left)), m_right(std::move(right))
{}

std::string BinMathNode::print() const
{
    std::string s;

    switch (type())
    {
    case NodeType::Add:
        s = "+";
        break;
    case NodeType::Subtract:
        s = "-";
        break;
    case NodeType::Multiply:
        s = "*";
        break;
    case NodeType::Divide:
        s = "/";
        break;
    default:
        break;
    }

    return "(" + m_left->print() + " " + s + " " + m_right->print() + ")";
}

Utils::StatusWithReason BinMathNode::prepare(PointLayoutPtr l)
{
    auto status = m_left->prepare(l);
    if (status)
        status = m_right->prepare(l);
    return status;
}

Result BinMathNode::eval(PointRef& p) const
{
    double l = m_left->eval(p).m_dval;
    double r = m_right->eval(p).m_dval;

    switch (type())
    {
    case NodeType::Add:
        return l + r;
    case NodeType::Subtract:
        return l - r;
    case NodeType::Multiply:
        return l * r;
    case NodeType::Divide:
        if (r == 0)
            return std::numeric_limits<double>::quiet_NaN();
        return l / r;
    default:
        break;
    }
    assert(false);
    return 0.0;
}

//
// Bool node
//
BoolNode::BoolNode(NodeType type, NodePtr left, NodePtr right) :
    LogicalNode(type), m_left(std::move(left)), m_right(std::move(right))
{}

std::string BoolNode::print() const
{
    std::string s;
    switch (type())
    {
    case NodeType::And:
        s = "&&";
        break;
    case NodeType::Or:
        s = "||";
        break;
    default:
        break;
    }

    return "(" + m_left->print() + " " + s + " " + m_right->print() + ")";
}

Utils::StatusWithReason BoolNode::prepare(PointLayoutPtr l)
{
    auto status = m_left->prepare(l);
    if (status)
        status = m_right->prepare(l);
    return status;
}

Result BoolNode::eval(PointRef& p) const
{
    bool l = m_left->eval(p).m_bval;
    bool r = m_right->eval(p).m_bval;
    switch (type())
    {
    case NodeType::And:
        return l && r;
    case NodeType::Or:
        return l || r;
    default:
        break;
    }
    assert(false);
    return false;

}

//
// CompareNode
//
CompareNode::CompareNode(NodeType type, NodePtr left, NodePtr right) :
    LogicalNode(type), m_left(std::move(left)), m_right(std::move(right))
{}

Utils::StatusWithReason CompareNode::prepare(PointLayoutPtr l)
{
    auto status = m_left->prepare(l);
    if (status)
        status = m_right->prepare(l);
    return status;
}

std::string CompareNode::print() const
{
    std::string s;
    switch (type())
    {
    case NodeType::Equal:
        s = "==";
        break;
    case NodeType::NotEqual:
        s = "!=";
        break;
    case NodeType::Greater:
        s = ">";
        break;
    case NodeType::GreaterEqual:
        s = ">=";
        break;
    case NodeType::Less:
        s = "<";
        break;
    case NodeType::LessEqual:
        s = "<=";
        break;
    default:
        break;
    }
    return "(" + m_left->print() + s + m_right->print() + ")";
}

Result CompareNode::eval(PointRef& p) const
{
    double l = m_left->eval(p).m_dval;
    double r = m_right->eval(p).m_dval;
    switch (type())
    {
    case NodeType::Equal:
        return l == r;
    case NodeType::NotEqual:
        return l != r;
    case NodeType::Less:
        return l < r;
    case NodeType::LessEqual:
        return l <= r;
    case NodeType::Greater:
        return l > r;
    case NodeType::GreaterEqual:
        return l >= r;
    default:
        break;
    }
    assert(false);
    return false;
}

//
// ConstValueNode
//
ConstValueNode::ConstValueNode(double d) : ValueNode(NodeType::Value), m_val(d)
{}


std::string ConstValueNode::print() const
{
    return std::to_string(m_val);
}

Utils::StatusWithReason ConstValueNode::prepare(PointLayoutPtr l)
{
    return true;
}

Result ConstValueNode::eval(PointRef&) const
{
    return m_val;
}

double ConstValueNode::value() const
{
    return m_val;
}

//
// ConstLogicalNode
//
ConstLogicalNode::ConstLogicalNode(bool b) :
    LogicalNode(NodeType::Value), m_val(b)
{}


std::string ConstLogicalNode::print() const
{
    return m_val ? "true" : "false";
}

Utils::StatusWithReason ConstLogicalNode::prepare(PointLayoutPtr l)
{
    return true;
}

Result ConstLogicalNode::eval(PointRef&) const
{
    return m_val;
}

bool ConstLogicalNode::value() const
{
    return m_val;
}

//
// Var Node
//
VarNode::VarNode(const std::string& s) : ValueNode(NodeType::Identifier),
    m_name(s), m_id(Dimension::Id::Unknown)
{}

std::string VarNode::print() const
{
    return m_name;
}

Result VarNode::eval(PointRef& p) const
{
    return p.getFieldAs<double>(m_id);
}

Dimension::Id VarNode::eval() const
{
    return m_id;
}

Utils::StatusWithReason VarNode::prepare(PointLayoutPtr l)
{
    m_id = l->findDim(m_name);
    if (m_id == Dimension::Id::Unknown)
        return { -1, "Unknown dimension '" + m_name + "' in assignment." };
    return true;
}

//
// Expression
//

Expression::Expression()
{}

Expression::~Expression()
{}

// This is a strange copy ctor that ignores the source.  At this point we
// don't need it to do anything, but we do need the an expression to
// be copyable in order to be used by ProgramArgs.
// In order to copy, we'd actually have to deep-copy the nodes, but there is
// no way to do that right now.
Expression::Expression(const Expression& expr)
{
    if (m_nodes.size())
        throw pdal_error("Attempting to copy expression with nodes.");
}

// noexcept is important here. Without, vector won't call the move ctor when
//   resizing. Instead, it will call the copy ctor, which is bad, since our
//   copy ctor is busted.
Expression::Expression(Expression&& expr) noexcept :
    m_error(expr.m_error), m_nodes(std::move(expr.m_nodes))
{}

Expression& Expression::operator=(Expression&& expr)
{
    m_error = expr.m_error;
    m_nodes = std::move(expr.m_nodes);
    return *this;
}

// This is a strange assignment operator that ignores the source.  At this point we
// don't need it to do anything, but we do need the an expression to
// be assigned in order to be used by ProgramArgs.
// In order to copy, we'd actually have to deep-copy the nodes, but there is
// no way to do that right now.
Expression& Expression::operator=(const Expression& expr)
{
    if (m_nodes.size())
        throw pdal_error("Attempting to assign expression with nodes.");
    return *this;
}

bool Expression::valid() const
{
    return m_nodes.size();
}

void Expression::clear()
{
    std::stack<NodePtr> empty;
    m_nodes.swap(empty);
    m_error.clear();
}

std::string Expression::error() const
{
    return m_error;
}

std::string Expression::print() const
{
    if (m_nodes.size())
        return m_nodes.top()->print();
    return std::string();
}

NodePtr Expression::popNode()
{
    NodePtr n(std::move(m_nodes.top()));
    m_nodes.pop();
    return n;
}

void Expression::pushNode(NodePtr node)
{
    m_nodes.push(std::move(node));
}

Node *Expression::topNode()
{
    return m_nodes.size() ? m_nodes.top().get() : nullptr;
}

const Node *Expression::topNode() const
{
    return m_nodes.size() ? m_nodes.top().get() : nullptr;
}

Utils::StatusWithReason Expression::prepare(PointLayoutPtr layout)
{
    if (m_nodes.size())
        return m_nodes.top()->prepare(layout);
    return true;
}

std::ostream& operator<<(std::ostream& out, const Expression& expr)
{
    out << expr.print();
    return out;
}

} // namespace expr
} // namespace pdal

