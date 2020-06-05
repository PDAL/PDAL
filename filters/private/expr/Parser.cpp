#include "Parser.hpp"

namespace pdal
{
namespace expr
{

bool Parser::parse(const std::string& s)
{
    std::stack<NodePtr> nodes;
    nodes.swap(m_nodes);

    m_lexer.lex(s);
    return expression();
}


void Parser::dump()
{
    std::cout << m_nodes.top()->print() << "\n";
}

/**
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
**/


Token Parser::popToken(TokenClass cls)
{
    return m_lexer.get(cls);
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
        Token tok = popToken(TokenClass::Or);
        if (!tok)
            return true;

        if (!andexpr())
        {
            m_error = "OR Operator not followed by rvalue.";
            return false;
        }

        NodePtr right = popNode();
        NodePtr left = popNode();

        ValNode *leftVal = dynamic_cast<ValNode *>(left.get());
        ValNode *rightVal = dynamic_cast<ValNode *>(right.get());
        if (leftVal && rightVal)
        {
            double v = leftVal->value() || rightVal->value();
            pushNode(NodePtr(new ValNode(v)));
        }
        else
            pushNode(NodePtr(new BoolNode(NodeType::Or,
                std::move(left), std::move(right))));
    }
    return true;
}

bool Parser::andexpr()
{
    if (!compareexpr())
        return false;

    while (true)
    {
        Token tok = popToken(TokenClass::And);
        if (!tok)
            return true;

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
            double v = leftVal->value() && rightVal->value();
            pushNode(NodePtr(new ValNode(v)));
        }
        else
            pushNode(NodePtr(new BoolNode(NodeType::And,
                std::move(left), std::move(right))));
    }
    return true;
}

//ABELL - This treats == and >= at the same precendence level.  In C++,
// <, >, <=, >= come before ==, !=
bool Parser::compareexpr()
{
    if (!addexpr())
        return false;

    while (true)
    {
        Token tok = popToken(TokenClass::Compare);
        if (!tok)
            return true;

        NodeType type;
        if (tok.type() == TokenType::Equal)
            type = NodeType::Equal;
        else if (tok.type() == TokenType::NotEqual)
            type = NodeType::NotEqual;
        else if (tok.type() == TokenType::Greater)
            type = NodeType::Greater;
        else if (tok.type() == TokenType::GreaterEqual)
            type = NodeType::GreaterEqual;
        else if (tok.type() == TokenType::Less)
            type = NodeType::Less;
        else if (tok.type() == TokenType::LessEqual)
            type = NodeType::LessEqual;

        if (!addexpr())
        {
            m_error = "COMP Operator not followed by rvalue.";
            return false;
        }

        NodePtr right = popNode();
        NodePtr left = popNode();

        ValNode *leftVal = dynamic_cast<ValNode *>(left.get());
        ValNode *rightVal = dynamic_cast<ValNode *>(right.get());
        if (leftVal && rightVal)
        {
            double v(0);
            if (type == NodeType::Equal)
                v = (leftVal->value() == rightVal->value());
            else if (type == NodeType::NotEqual)
                v = (leftVal->value() != rightVal->value());
            else if (type == NodeType::Greater)
                v = (leftVal->value() > rightVal->value());
            else if (type == NodeType::GreaterEqual)
                v = (leftVal->value() >= rightVal->value());
            else if (type == NodeType::Less)
                v = (leftVal->value() < rightVal->value());
            else if (type == NodeType::LessEqual)
                v = (leftVal->value() <= rightVal->value());
            pushNode(NodePtr(new ValNode(v)));
        }
        else
            pushNode(NodePtr(new BoolNode(type,
                std::move(left), std::move(right))));
    }
    return true;
}

bool Parser::addexpr()
{
    if (!multexpr())
        return false;

    while (true)
    {
        Token tok = popToken(TokenClass::Add);
        if (!tok)
            return true;

        NodeType type;
        if (tok.type() == TokenType::Add)
            type = NodeType::Add;
        else if (tok.type() == TokenType::Subtract)
            type = NodeType::Subtract;

        if (!multexpr())
        {
            m_error = "ADD Operator not followed by rvalue.";
            return false;
        }

        NodePtr right(popNode());
        NodePtr left(popNode());

        ValNode *leftVal = dynamic_cast<ValNode *>(left.get());
        ValNode *rightVal = dynamic_cast<ValNode *>(right.get());
        if (leftVal && rightVal)
        {
            double v = (type == NodeType::Add) ?
                leftVal->value() + rightVal->value() :
                leftVal->value() - rightVal->value();
            pushNode(NodePtr(new ValNode(v)));
        }
        else
            pushNode(NodePtr(new BinNode(type,
                std::move(left), std::move(right))));
    }
    return true;
}

bool Parser::multexpr()
{
    if (!primary())
        return false;

    while (true)
    {
        Token tok = popToken(TokenClass::Multiply);
        if (!tok)
            return true;

        NodeType type;
        if (tok.type() == TokenType::Multiply)
            type = NodeType::Multiply;
        else if (tok.type() == TokenType::Divide)
            type = NodeType::Divide;


        if (!primary())
        {
            m_error = "MULT Operator not followed by rvalue.";
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
            pushNode(NodePtr(new BinNode(type,
                std::move(left), std::move(right))));
    }
    return true;
}

bool Parser::primary()
{
    Token tok = popToken(TokenClass::Primary);
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

    return parexpr();
}


bool Parser::parexpr()
{
    Token tok = popToken(TokenClass::Lparen);
    if (!tok)
        return false;

    if (!expression())
        return false;
    tok = popToken(TokenClass::Rparen);
    if (!tok)
        return false;
    return true;
}

} // namespace expr
} // namespace pdal
