#pragma once

#include <stack>
#include <string>
#include <memory>

#include <pdal/util/Utils.hpp>

namespace pdal
{
namespace expr
{

enum class NodeType
{
    And,
    Or,
    Add,
    Subtract,
    Multiply,
    Divide,
    Not,
    Equal,
    NotEqual,
    Greater,
    GreaterEqual,
    Less,
    LessEqual,
    Negative,
    Value,
    Identifier,
    None
};

class Node
{
protected:
    Node(NodeType type);

public:
    virtual ~Node();
    NodeType type() const;

    virtual std::string print() const = 0;
    /**
    virtual void prepare(PointLayoutPtr l) = 0;
    virtual double eval(PointRef& p) const = 0;
    **/

private:
    NodeType m_type;

protected:
    size_t m_pos;
    size_t m_level;
};
using NodePtr = std::unique_ptr<Node>;

class UnNode : public Node
{
public:
    UnNode(NodeType type, NodePtr sub);

    virtual std::string print() const;

private:
    NodePtr m_sub;
};

class BinNode : public Node
{
public:
    BinNode(NodeType type, NodePtr left, NodePtr right);

    virtual std::string print() const;

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

private:
    NodePtr m_left;
    NodePtr m_right;
};

class BoolNode : public Node
{
public:
    BoolNode(NodeType type, NodePtr left, NodePtr right);

    virtual std::string print() const;

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

private:
    NodePtr m_left;
    NodePtr m_right;
};

class ValNode : public Node
{
public:
    ValNode(double d);

    virtual std::string print() const;

    /**
    virtual void prepare(PointLayoutPtr l)
    {}

    virtual double eval(PointRef&) const
    { return m_val; }
    **/

    double value() const;

private:
    double m_val;
};

class VarNode : public Node
{
public:
    VarNode(const std::string& s);

    virtual std::string print() const;

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

private:
    std::string m_name;
//    Dimension::Id m_id;
};

class Expression
{
public:
    Expression();
    Expression(const Expression& expr);
    Expression& operator=(const Expression& expr);
    ~Expression();

    void clear();
    bool parse(const std::string& s);
    std::string error() const;
    std::string print() const;
    NodePtr popNode();
    void pushNode(NodePtr node);

private:
    std::string m_error;
    std::stack<NodePtr> m_nodes;

    friend std::ostream& operator<<(std::ostream& out, const Expression& expr);
};

} // namespace expr

namespace Utils
{

template<>
inline StatusWithReason fromString(const std::string& from,
    pdal::expr::Expression& expr)
{
    bool ok = expr.parse(from);
    return { ok ? 0 : -1, expr.error() };
}

} // namespace Util

} // namespace pdal
