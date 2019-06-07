/******************************************************************************
* Copyright (c) 2019, Andrew Bell (andrew.bell.ia@gmail.com)
*
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following
* conditions are met:
*
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in
*       the documentation and/or other materials provided
*       with the distribution.
*     * The name of Andrew Bell may not be used to endorse or promote
*       products derived from this software without specific prior
*       written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
* OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
* AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
* OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
* OF SUCH DAMAGE.
****************************************************************************/

#pragma once

#include <stack>

#include <pdal/PointLayout.hpp>
#include <pdal/PointRef.hpp>

#include "Lexer.hpp"

namespace pdal
{
namespace expr
{

enum class NodeType
{
    Plus,
    Minus,
    Multiply,
    Divide,
    Equal,
    Inequality,
    Greater,
    GreaterOrEqual,
    Less,
    LessOrEqual,
    In,
    And,
    Or,
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

    virtual void prepare(PointLayoutPtr l) = 0;
    virtual double eval(PointRef& p) const = 0;

private:
    NodeType m_type;
};
using NodePtr = std::unique_ptr<Node>;

class BinNode : public Node
{
public:
    BinNode(NodeType type, NodePtr left, NodePtr right) :
        Node(type), m_left(std::move(left)), m_right(std::move(right))
    {}

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
        case NodeType::Plus:
            return l + r;
        case NodeType::Minus:
            return l - r;
        case NodeType::Multiply:
            return l * r;
        case NodeType::Divide:
            return l / r;
        case NodeType::Equal:
            return l == r;
        case NodeType::Inequality:
            return l != r;
        case NodeType::Greater:
            return l > r;
        case NodeType::GreaterOrEqual:
            return l >= r;
        case NodeType::Less:
            return l < r;
        case NodeType::LessOrEqual:
            return l <= r;
        default:
            return 0;
        }
    }

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

    virtual void prepare(PointLayoutPtr l)
    {
        m_left->prepare(l);
        m_right->prepare(l);
    }

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

private:
    NodePtr m_left;
    NodePtr m_right;
};

class ValNode : public Node
{
public:
    ValNode(double d) : Node(NodeType::Value), m_val(d)
    {}

    virtual void prepare(PointLayoutPtr l)
    {}

    virtual double eval(PointRef&) const
    { return m_val; }

    double value() const
    { return m_val; }

private:
    double m_val;
};

class ValInNode : public Node
{
public:
    ValInNode(NodeType type, NodePtr left, std::vector<NodePtr>& right) :
        Node(type), m_left(std::move(left))
    {
        for (NodePtr& r : right)
        {
            m_right.push_back(std::move(r));
        }
    }

    virtual void prepare(PointLayoutPtr l)
    {
        m_left->prepare(l);

        for (NodePtr& r : m_right)
        {
            r->prepare(l);
        }
    }

    virtual double eval(PointRef& p) const
    {
        double v = m_left->eval(p);

        for (const NodePtr& r : m_right)
        {
            if (v == r->eval(p))
                return 1;
        }
        return 0;
    }

private:
    NodePtr m_left;
    std::vector<NodePtr> m_right;
};

class VarNode : public Node
{
public:
    VarNode(const std::string& s) : Node(NodeType::Variable), m_name(s),
        m_id(Dimension::Id::Unknown)
    {}

    virtual void prepare(PointLayoutPtr l)
    {
        m_id = l->findDim(m_name);
        if (m_id == Dimension::Id::Unknown)
            std::cerr << "Unknown dimension '" << m_name << "' in assigment.";
    }

    virtual double eval(PointRef& p) const
    { return p.getFieldAs<double>(m_id); }

private:
    std::string m_name;
    Dimension::Id m_id;
};

class Parser
{
public:
    bool parse(const std::string& s);
    std::string error() const
    { return m_error; }
    void prepare(PointLayoutPtr l);
    double eval(PointRef& p) const;

private:
    void pushNode(std::unique_ptr<Node> node)
    { m_nodes.push(std::move(node)); }

    NodePtr popNode()
    {
        NodePtr n(std::move(m_nodes.top()));
        m_nodes.pop();
        return n;
    }

    Token popToken(TokenClass cls = TokenClass::Any);
    void pushToken(const Token& tok);
    bool expression();
    bool orexpr();
    bool andexpr();
    bool compareexpr();
    bool addexpr();
    bool multexpr();
    bool primary();
    bool parexpr();
    bool arrayexpr();

    Lexer m_lexer;
    std::stack<NodePtr> m_nodes;
    std::stack<Token> m_tokens;
    std::string m_error;
};

} // namespace expr
} // namespace pdal
