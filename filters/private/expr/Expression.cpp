#include "Expression.hpp"
#include "Parser.hpp"

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


UnNode::UnNode(NodeType type, NodePtr sub) :
    Node(type), m_sub(std::move(sub))
{}

std::string UnNode::print() const
{
    return "!(" + m_sub->print() + ")";
}

BinNode::BinNode(NodeType type, NodePtr left, NodePtr right) :
    Node(type), m_left(std::move(left)), m_right(std::move(right))
{}

std::string BinNode::print() const
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

/**
virtual void prepare(PointLayoutPtr l)
{
m_left->prepare(l);
m_right->prepare(l);
}

virtual double eval(PointRef& p) const
{
double l = m_left->eval(p);
double r = m_right->eval(p);
switch (type())
{
case NodeType::Add:
return l + r;
case NodeType::Subtract:
return l - r;
case NodeType::Multiply:
return l * r;
case NodeType::Divide:
return l / r;
default:
return 0;
}
}
**/

BoolNode::BoolNode(NodeType type, NodePtr left, NodePtr right) :
    Node(type), m_left(std::move(left)), m_right(std::move(right))
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

    return "(" + m_left->print() + " " + s + " " + m_right->print() + ")";
}

    /**
    virtual void prepare(PointLayoutPtr l)
    {
        m_left->prepare(l);
        m_right->prepare(l);
    }
    **/

    /**
    virtual double eval(PointRef& p) const
    {
        switch (type())
        {
        case NodeType::And:
            return m_left->eval(p) && m_right->eval(p);
        case NodeType::Or:
            return m_left->eval(p) || m_right->eval(p);
        default:
            return 0;
        }
    }
    **/

ValNode::ValNode(double d) : Node(NodeType::Value), m_val(d)
{}

std::string ValNode::print() const
{
    return std::to_string(m_val);
}

    /**
    virtual void prepare(PointLayoutPtr l)
    {}

    virtual double eval(PointRef&) const
    { return m_val; }
    **/

double ValNode::value() const
{ return m_val; }

VarNode::VarNode(const std::string& s) : Node(NodeType::Identifier), m_name(s)
//    , m_id(Dimension::Id::Unknown)
{}

std::string VarNode::print() const
{ return m_name; }

    /**
    virtual void prepare(PointLayoutPtr l)
    {
        m_id = l->findDim(m_name);
        if (m_id == Dimension::Id::Unknown)
            std::cerr << "Unknown dimension '" << m_name << "' in assigment.";
    }

    virtual double eval(PointRef& p) const
    { return p.getFieldAs<double>(m_id); }
    **/

Expression::Expression()
{}

Expression::~Expression()
{}

void Expression::clear()
{
    std::stack<NodePtr> empty;
    m_nodes.swap(empty);
    m_error.clear();
}

bool Expression::parse(const std::string& s)
{
    clear();
    Parser p(*this);
    bool ok = p.parse(s);
    if (!ok)
        m_error = p.error();
    return ok;
}

std::string Expression::error() const
{ return m_error; }

void Expression::print() const
{ std::cout << m_nodes.top()->print() << "\n"; }

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

} // namespace expr
} // namespace pdal
