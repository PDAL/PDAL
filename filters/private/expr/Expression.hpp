#pragma once

#include <stack>
#include <string>
#include <memory>

#include <pdal/Dimension.hpp>
#include <pdal/PointLayout.hpp>
#include <pdal/PointRef.hpp>
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

struct Result
{
    Result(double d)
    { m_dval = d; m_bval = false; }

    Result(bool b)
    { m_bval = b; m_dval = 0; }

    enum class Type
    {
        Bool,
        Val
    };

    double m_dval;
    bool m_bval;
    Type m_type;
};

class Node
{
protected:
    Node(NodeType type);

public:
    virtual ~Node();
    NodeType type() const;

    virtual std::string print() const = 0;
    virtual Utils::StatusWithReason prepare(PointLayoutPtr l) = 0;
    virtual Result eval(PointRef& p) const = 0;
    virtual bool isBool() const = 0;
    virtual bool isValue() const
    { return !isBool(); }

private:
    NodeType m_type;

protected:
    size_t m_pos;
    size_t m_level;
};
using NodePtr = std::unique_ptr<Node>;

class LogicalNode : public Node
{
public:
    LogicalNode(NodeType type) : Node(type)
    {}

    virtual bool isBool() const
    { return true; }
};

class ValueNode : public Node
{
public:
    ValueNode(NodeType type) : Node(type)
    {}

    virtual bool isBool() const
    { return false; }
};

class BinMathNode : public ValueNode
{
public:
    BinMathNode(NodeType type, NodePtr left, NodePtr right);

    virtual std::string print() const;
    virtual Utils::StatusWithReason prepare(PointLayoutPtr l);
    virtual Result eval(PointRef& p) const;

private:
    NodePtr m_left;
    NodePtr m_right;
};

class UnMathNode : public ValueNode
{
public:
    UnMathNode(NodeType type, NodePtr sub);

    virtual std::string print() const;
    virtual Utils::StatusWithReason prepare(PointLayoutPtr l);
    virtual Result eval(PointRef& p) const;

private:
    NodePtr m_sub;
};

class NotNode : public LogicalNode
{
public:
    NotNode(NodeType type, NodePtr sub);

    virtual std::string print() const;
    virtual Utils::StatusWithReason prepare(PointLayoutPtr l);
    virtual Result eval(PointRef& p) const;

private:
    NodePtr m_sub;
};

class BoolNode : public LogicalNode
{
public:
    BoolNode(NodeType type, NodePtr left, NodePtr right);

    virtual std::string print() const;
    virtual Utils::StatusWithReason prepare(PointLayoutPtr l);
    virtual Result eval(PointRef& p) const;

private:
    NodePtr m_left;
    NodePtr m_right;
};

class CompareNode : public LogicalNode
{
public:
    CompareNode(NodeType type, NodePtr left, NodePtr right);

    virtual std::string print() const;
    virtual Utils::StatusWithReason prepare(PointLayoutPtr l);
    virtual Result eval(PointRef& p) const;

private:
    NodePtr m_left;
    NodePtr m_right;
};

class ConstValueNode : public ValueNode
{
public:
    ConstValueNode(double d);

    virtual std::string print() const;
    virtual Utils::StatusWithReason prepare(PointLayoutPtr l);
    virtual Result eval(PointRef&) const;

    double value() const;

private:
    double m_val;
};

class ConstLogicalNode : public LogicalNode
{
public:
    ConstLogicalNode(bool b);

    virtual std::string print() const;
    virtual Utils::StatusWithReason prepare(PointLayoutPtr l);
    virtual Result eval(PointRef&) const;

    bool value() const;

private:
    bool m_val;
};

class VarNode : public ValueNode
{
public:
    VarNode(const std::string& s);

    virtual std::string print() const;
    virtual Utils::StatusWithReason prepare(PointLayoutPtr l);
    virtual Result eval(PointRef& p) const;
    Dimension::Id eval() const;

private:
    std::string m_name;
    Dimension::Id m_id;
};

class Expression
{
public:
    Expression();
    virtual ~Expression();
    Expression(const Expression& expr);
    Expression(Expression&& expr) noexcept;
    Expression& operator=(Expression&& expr);
    Expression& operator=(const Expression& expr);

    void clear();
    bool valid() const;
    std::string error() const;
    NodePtr popNode();
    void pushNode(NodePtr node);
    Node *topNode();
    const Node *topNode() const;
    virtual std::string print() const;
    virtual Utils::StatusWithReason prepare(PointLayoutPtr layout) = 0;

private:
    std::string m_error;
    std::stack<NodePtr> m_nodes;

    friend std::ostream& operator<<(std::ostream& out, const Expression& expr);
};

} // namespace expr
} // namespace pdal
