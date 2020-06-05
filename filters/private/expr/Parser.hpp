#pragma once

#include <queue>
#include <stack>
#include <iostream>
#include <iomanip>

#include "Lexer.hpp"

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
    Equal,
    NotEqual,
    Greater,
    GreaterEqual,
    Less,
    LessEqual,
    Value,
    Variable
};

class Node
{
protected:
    Node(NodeType type) : m_type(type)
    {}

public:
    virtual ~Node()
    {}
    NodeType type() const
    { return m_type; }

    virtual std::string print() = 0;
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

class BinNode : public Node
{
public:
    BinNode(NodeType type, NodePtr left, NodePtr right) :
        Node(type), m_left(std::move(left)), m_right(std::move(right))
    {}

    virtual std::string print()
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

private:
    NodePtr m_left;
    NodePtr m_right;
};

class BoolNode : public Node
{
public:
    BoolNode(NodeType type, NodePtr left, NodePtr right) :
        Node(type), m_left(std::move(left)), m_right(std::move(right))
    {}

    virtual std::string print()
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

private:
    NodePtr m_left;
    NodePtr m_right;
};

class ValNode : public Node
{
public:
    ValNode(double d) : Node(NodeType::Value), m_val(d)
    {}

    virtual std::string print()
    {
        return std::to_string(m_val);
    }

    /**
    virtual void prepare(PointLayoutPtr l)
    {}

    virtual double eval(PointRef&) const
    { return m_val; }
    **/

    double value() const
    { return m_val; }

private:
    double m_val;
};

class VarNode : public Node
{
public:
    VarNode(const std::string& s) : Node(NodeType::Variable), m_name(s)
//    , m_id(Dimension::Id::Unknown)
    {}

    virtual std::string print()
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

private:
    std::string m_name;
//    Dimension::Id m_id;
};

class Parser
{
public:
    bool parse(const std::string& s);
    std::string error() const
    { return m_error; }
    void dump();
    /**
    void prepare(PointLayoutPtr l);
    double eval(PointRef& p) const;
    **/

private:
    void pushNode(std::unique_ptr<Node> node)
    { m_nodes.push(std::move(node)); }

    NodePtr popNode()
    {
        NodePtr n(std::move(m_nodes.top()));
        m_nodes.pop();
        return n;
    }

    Token popToken(TokenClass cls);
    bool expression();
    bool orexpr();
    bool andexpr();
    bool compareexpr();
    bool addexpr();
    bool multexpr();
    bool primary();
    bool parexpr();

    Lexer m_lexer;
    std::stack<NodePtr> m_nodes;
    std::string m_error;
};

} // namespace expr
} // namespace pdal
