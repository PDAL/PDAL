#include "Parser.hpp"

namespace pdal
{
namespace expr
{

bool Parser::parse(const std::string& s)
{
    m_expression.clear();

    m_lexer.reset(s);
    m_error.clear();
    bool ok = expression();
    if (ok)
    {
        // If we're at the end, we should have exhausted all tokens.
        Token tok = m_lexer.get();
        if (tok != TokenType::Eof)
        {
            m_error = "Found '" + tok.sval() + "' after valid expression.";
            return false;
        }
    }
    return ok;
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


Token Parser::curToken() const
{
    return m_curTok;
}

bool Parser::match(TokenType type)
{
    Token t = m_lexer.get();
    if (t.type() == type)
    {
        m_curTok = t;
        return true;
    }
    m_lexer.put(t);
    return false;
}

void Parser::setError(const std::string& err)
{
    if (m_error.empty())
        m_error = err;
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
        if (!match(TokenType::Or))
            return true;

        if (!andexpr())
        {
            setError("Expected expression following '||'.");
            return false;
        }

        NodePtr right = m_expression.popNode();
        NodePtr left = m_expression.popNode();

        ValNode *leftVal = dynamic_cast<ValNode *>(left.get());
        ValNode *rightVal = dynamic_cast<ValNode *>(right.get());
        if (leftVal && rightVal)
        {
            double v = leftVal->value() || rightVal->value();
            m_expression.pushNode(NodePtr(new ValNode(v)));
        }
        else
            m_expression.pushNode(NodePtr(new BoolNode(NodeType::Or,
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
        if (!match(TokenType::And))
            return true;

        if (!compareexpr())
        {
            setError("Expected expression following '&&'.");
            return false;
        }

        NodePtr right = m_expression.popNode();
        NodePtr left = m_expression.popNode();

        ValNode *leftVal = dynamic_cast<ValNode *>(left.get());
        ValNode *rightVal = dynamic_cast<ValNode *>(right.get());
        if (leftVal && rightVal)
        {
            double v = leftVal->value() && rightVal->value();
            m_expression.pushNode(NodePtr(new ValNode(v)));
        }
        else
            m_expression.pushNode(NodePtr(new BoolNode(NodeType::And,
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
        NodeType type = NodeType::None;

        if (match(TokenType::Equal))
            type = NodeType::Equal;
        else if (match(TokenType::NotEqual))
            type = NodeType::NotEqual;
        else if (match(TokenType::Greater))
            type = NodeType::Greater;
        else if (match(TokenType::GreaterEqual))
            type = NodeType::GreaterEqual;
        else if (match(TokenType::Less))
            type = NodeType::Less;
        else if (match(TokenType::LessEqual))
            type = NodeType::LessEqual;
        else
            return true;

        if (!addexpr())
        {
            setError("Expected expression following '" +
                curToken().sval() + "'.");
            return false;
        }

        NodePtr right = m_expression.popNode();
        NodePtr left = m_expression.popNode();
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
            m_expression.pushNode(NodePtr(new ValNode(v)));
        }
        else
            m_expression.pushNode(NodePtr(new BoolNode(type,
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
        NodeType type;

        if (match(TokenType::Plus))
            type = NodeType::Add;
        else if (match(TokenType::Dash))
            type = NodeType::Subtract;
        else
            return true;

        if (!multexpr())
        {
            setError("Expected expression following '" +
                curToken().sval() + "'.");
            return false;
        }

        NodePtr right = m_expression.popNode();
        NodePtr left = m_expression.popNode();

        ValNode *leftVal = dynamic_cast<ValNode *>(left.get());
        ValNode *rightVal = dynamic_cast<ValNode *>(right.get());
        if (leftVal && rightVal)
        {
            double v = (type == NodeType::Add) ?
                leftVal->value() + rightVal->value() :
                leftVal->value() - rightVal->value();
            m_expression.pushNode(NodePtr(new ValNode(v)));
        }
        else
            m_expression.pushNode(NodePtr(new BinNode(type,
                std::move(left), std::move(right))));
    }
    return true;
}

bool Parser::multexpr()
{
    if (!notexpr())
        return false;

    while (true)
    {
        NodeType type;
        if (match(TokenType::Asterisk))
            type = NodeType::Multiply;
        else if (match(TokenType::Slash))
            type = NodeType::Divide;
        else
            return true;

        if (!notexpr())
        {
            setError("Expected expression following '" +
                curToken().sval() + "'.");
            return false;
        }

        NodePtr right = m_expression.popNode();
        NodePtr left = m_expression.popNode();

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
                    setError("Divide by 0.");
                    return false;
                }
                v = leftVal->value() / rightVal->value();
            }
            m_expression.pushNode(NodePtr(new ValNode(v)));
        }
        else
            m_expression.pushNode(NodePtr(new BinNode(type,
                std::move(left), std::move(right))));
    }
    return true;
}

bool Parser::notexpr()
{
    if (!match(TokenType::Not))
        return uminus();

    if (!uminus())
    {
        setError("Expected expression following '!'.");
        return false;
    }

    NodePtr sub = m_expression.popNode();
    ValNode *node = dynamic_cast<ValNode *>(sub.get());
    if (node)
    {
        double v = !node->value();
        m_expression.pushNode(NodePtr(new ValNode(v)));
    }
    else
        m_expression.pushNode(
            NodePtr(new UnNode(NodeType::Not, std::move(sub))));
    return true;
}

bool Parser::uminus()
{
    if (!match(TokenType::Dash))
        return primary();

    if (!primary())
    {
        setError("Expecting expression following '-'.");
        return false;
    }

    NodePtr sub = m_expression.popNode();
    ValNode *node = dynamic_cast<ValNode *>(sub.get());
    if (node)
    {
        double v = -(node->value());
        m_expression.pushNode(NodePtr(new ValNode(v)));
    }
    else
        m_expression.pushNode(
            NodePtr(new UnNode(NodeType::Negative, std::move(sub))));
    return true;
}

bool Parser::primary()
{
    if (match(TokenType::Number))
    {
        m_expression.pushNode(NodePtr(new ValNode(curToken().dval())));
        return true;
    }
    else if (match(TokenType::Identifier))
    {
        m_expression.pushNode(NodePtr(new VarNode(curToken().sval())));
        return true;
    }

    return parexpr();
}


bool Parser::parexpr()
{
    if (!match(TokenType::Lparen))
        return false;

    if (!expression())
    {
        setError("Expected expression following '('.");
        return false;
    }

    if (!match(TokenType::Rparen))
    {
        setError("Expected ')' following expression.");
        return false;
    }
    return true;
}

} // namespace expr
} // namespace pdal
