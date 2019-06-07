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

#include "Parser.hpp"

namespace pdal
{
namespace expr
{

bool Parser::parse(const std::string& s)
{
    m_lexer.lex(s);
    return expression();
}


void Parser::prepare(PointLayoutPtr l)
{
    if (m_nodes.size() != 1)
    {
        std::cerr << "Can't prepare.  Node tree not properly parsed.\n";
        return;
    }
    m_nodes.top()->prepare(l);
}


double Parser::eval(PointRef& p) const
{
    if (m_nodes.size() != 1)
    {
        std::cerr << "Can't evaluate.  Node tree not properly parsed.\n";
        return 0;
    }
    return m_nodes.top()->eval(p);
}


// ABELL - This needs reworking since we may pop as another class.
Token Parser::popToken(TokenClass cls)
{
    if (m_tokens.empty())
        return m_lexer.get(cls);
    Token tok = m_tokens.top();
    m_tokens.pop();
    return tok;
}

void Parser::pushToken(const Token& tok)
{
    m_tokens.push(tok);
}


bool Parser::expression()
{
    if (!orexpr())
        return false;
    return true;
}

bool Parser::orexpr()
{
    if (!andexpr())
        return false;

    while (true)
    {
        Token tok = popToken(TokenClass::Operator);
        NodeType type;
        if (tok.type() == TokenType::Or)
            type = NodeType::Or;
        else
        {
            pushToken(tok);
            return true;
        }

        if (!andexpr())
        {
            m_error = "Operator not followed by rvalue.";
            return false;
        }

        NodePtr right = popNode();
        NodePtr left = popNode();

        ValNode *leftVal = dynamic_cast<ValNode *>(left.get());
        ValNode *rightVal = dynamic_cast<ValNode *>(right.get());
        if (leftVal && rightVal)
        {
            double v;
            if (type == NodeType::Or)
                v = leftVal->value() || rightVal->value();
            else
            {
                m_error = "Unknown boolean operator.";
                return false;
            }
            pushNode(NodePtr(new ValNode(v)));
        }
        else
            pushNode(NodePtr(new BoolNode(type, std::move(left),
                std::move(right))));
    }
    return true;
}

bool Parser::andexpr()
{
    if (!compareexpr())
        return false;

    while (true)
    {
        Token tok = popToken(TokenClass::Operator);
        NodeType type;
        if (tok.type() == TokenType::And)
            type = NodeType::And;
        else
        {
            pushToken(tok);
            return true;
        }

        if (!compareexpr())
        {
            m_error = "Operator not followed by rvalue.";
            return false;
        }

        NodePtr right = popNode();
        NodePtr left = popNode();

        ValNode *leftVal = dynamic_cast<ValNode *>(left.get());
        ValNode *rightVal = dynamic_cast<ValNode *>(right.get());
        if (leftVal && rightVal)
        {
            double v;
            if (type == NodeType::And)
                v = leftVal->value() && rightVal->value();
            else
            {
                m_error = "Unknown boolean operator.";
                return false;
            }
            pushNode(NodePtr(new ValNode(v)));
        }
        else
            pushNode(NodePtr(new BoolNode(type, std::move(left),
                std::move(right))));
    }
    return true;
}

bool Parser::compareexpr()
{
    if (!addexpr())
        return false;

    while (true)
    {
        Token tok = popToken(TokenClass::Operator);
        NodeType type;
        if (tok.type() == TokenType::Equal)
            type = NodeType::Equal;
        else if (tok.type() == TokenType::Inequality)
            type = NodeType::Inequality;
        else if (tok.type() == TokenType::Greater)
            type = NodeType::Greater;
        else if (tok.type() == TokenType::GreaterOrEqual)
            type = NodeType::GreaterOrEqual;
        else if (tok.type() == TokenType::Less)
            type = NodeType::Less;
        else if (tok.type() == TokenType::LessOrEqual)
            type = NodeType::LessOrEqual;
        else if (tok.type() == TokenType::In)
            type = NodeType::In;
        else
        {
            pushToken(tok);
            return true;
        }

        if (type == NodeType::In)
        {
            NodePtr left = popNode();

            if (!arrayexpr())
            {
                m_error = "In function not followed by array.";
                return false;
            }

            std::vector<NodePtr> right;

            while (m_nodes.size() > 1)
            {
                right.insert(right.begin(), popNode());
            }
            pushNode(NodePtr(new ValInNode(type, std::move(left), right)));
            return true;
        }
        if (!addexpr())
        {
            m_error = "Operator not followed by rvalue.";
            return false;
        }

        NodePtr right = popNode();
        NodePtr left = popNode();

        ValNode *leftVal = dynamic_cast<ValNode *>(left.get());
        ValNode *rightVal = dynamic_cast<ValNode *>(right.get());
        if (leftVal && rightVal)
        {
            double v;
            if (type == NodeType::Equal)
                v = leftVal->value() == rightVal->value();
            else if (type == NodeType::Inequality)
                v = leftVal->value() != rightVal->value();
            else if (type == NodeType::Greater)
                v = leftVal->value() > rightVal->value();
            else if (type == NodeType::GreaterOrEqual)
                v = leftVal->value() >= rightVal->value();
            else if (type == NodeType::Less)
                v = leftVal->value() < rightVal->value();
            else if (type == NodeType::LessOrEqual)
                v = leftVal->value() <= rightVal->value();
            else
            {
                m_error = "Unknown comparison operator.";
                return false;
            }
            pushNode(NodePtr(new ValNode(v)));
        }
        else
            pushNode(NodePtr(new BinNode(type, std::move(left),
                std::move(right))));
    }
    return true;
}

bool Parser::addexpr()
{
    if (!multexpr())
        return false;

    while (true)
    {
        Token tok = popToken(TokenClass::Operator);
        NodeType type;
        if (tok.type() == TokenType::Plus)
            type = NodeType::Plus;
        else if (tok.type() == TokenType::Minus)
            type = NodeType::Minus;
        else
        {
            pushToken(tok);
            return true;
        }

        if (!multexpr())
        {
            m_error = "Operator not followed by rvalue.";
            return false;
        }

        NodePtr right(popNode());
        NodePtr left(popNode());

        ValNode *leftVal = dynamic_cast<ValNode *>(left.get());
        ValNode *rightVal = dynamic_cast<ValNode *>(right.get());
        if (leftVal && rightVal)
        {
            double v =
                (type == NodeType::Plus) ?
                leftVal->value() + rightVal->value() :
                leftVal->value() - rightVal->value();
            pushNode(NodePtr(new ValNode(v)));
        }
        else
            pushNode(NodePtr(new BinNode(type, std::move(left),
                std::move(right))));
    }
    return true;
}

bool Parser::multexpr()
{
    if (!primary())
        return false;

    while (true)
    {
        Token tok = popToken(TokenClass::Operator);
        NodeType type;
        if (tok.type() == TokenType::Multiply)
            type = NodeType::Multiply;
        else if (tok.type() == TokenType::Divide)
            type = NodeType::Divide;
        else
        {
            pushToken(tok);
            return true;
        }

        if (!primary())
        {
            m_error = "Operator not followed by rvalue.";
            return false;
        }

        NodePtr right = popNode();
        NodePtr left = popNode();

        ValNode *leftVal = dynamic_cast<ValNode *>(left.get());
        ValNode *rightVal = dynamic_cast<ValNode *>(right.get());
        if (leftVal && rightVal)
        {
            double v;
            if (type == NodeType::Multiply)
                v = leftVal->value() * rightVal->value();
            else
            {
                if (rightVal->value() == 0.0)
                {
                    m_error = "Divide by 0.";
                    return false;
                }
                v = leftVal->value() / rightVal->value();
            }
            pushNode(NodePtr(new ValNode(v)));
        }
        else
            pushNode(NodePtr(new BinNode(type, std::move(left),
                std::move(right))));
    }
    return true;
}

bool Parser::primary()
{
    Token tok = popToken();
    if (tok.type() == TokenType::Number)
    {
        pushNode(NodePtr(new ValNode(tok.dval())));
        return true;
    }
    else if (tok.type() == TokenType::Dimension)
    {
        pushNode(NodePtr(new VarNode(tok.sval())));
        return true;
    }
    else if (tok.type() == TokenType::Eof)
        return true;

    pushToken(tok);
    return parexpr();
}


bool Parser::parexpr()
{
    Token tok = popToken();
    if (tok.type() != TokenType::Lparen)
    {
        pushToken(tok);
        return false;
    }

    if (!expression())
        return false;

    tok = popToken();
    if (tok.type() != TokenType::Rparen)
    {
        pushToken(tok);
        return false;
    }
    return true;
}

bool Parser::arrayexpr()
{
    Token tok = popToken();
    if (tok.type() != TokenType::Lparen)
    {
        pushToken(tok);
        return false;
    }

    while (expression() && (tok = popToken()).type() == TokenType::Comma)
    {
        if (m_error.size() > 0)
        {
            pushToken(tok);
            return false;
        }
    }

    if (tok.type() != TokenType::Rparen)
    {
        pushToken(tok);
        return false;
    }
    return true;
}

} // namespace expr
} // namespace pdal
