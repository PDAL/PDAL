#include "ConditionalParser.hpp"

namespace pdal
{
namespace expr
{

bool ConditionalParser::expression(Expression& expr)
{
    if (!orexpr(expr))
        return false;
    return true;
}

bool ConditionalParser::orexpr(Expression& expr)
{
    if (!andexpr(expr))
        return false;

    while (true)
    {
        if (!match(TokenType::Or))
            return true;

        if (!andexpr(expr))
        {
            setError("Expected expression following '||'.");
            return false;
        }

        NodePtr right = expr.popNode();
        NodePtr left = expr.popNode();

        if (left->isValue() || right->isValue())
        {
            setError("Can't apply '||' to numeric expression.");
            return false;
        }
        expr.pushNode(NodePtr(new BoolNode(NodeType::Or, std::move(left), std::move(right))));
    }
    return true;
}

bool ConditionalParser::andexpr(Expression& expr)
{
    if (!compareexpr(expr))
        return false;

    while (true)
    {
        if (!match(TokenType::And))
            return true;

        if (!compareexpr(expr))
        {
            setError("Expected expression following '&&'.");
            return false;
        }

        NodePtr right = expr.popNode();
        NodePtr left = expr.popNode();

        if (left->isValue() || right->isValue())
        {
            setError("Can't apply '&&' to numeric expression.");
            return false;
        }
        expr.pushNode(NodePtr(new BoolNode(NodeType::And, std::move(left), std::move(right))));
    }
    return true;
}

//ABELL - This treats == and >= at the same precendence level.  In C++,
// <, >, <=, >= come before ==, !=
bool ConditionalParser::compareexpr(Expression& expr)
{
    if (!addexpr(expr))
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

        if (!addexpr(expr))
        {
            setError("Expected expression following '" +
                curToken().sval() + "'.");
            return false;
        }

        NodePtr right = expr.popNode();
        NodePtr left = expr.popNode();
        ConstValueNode *leftVal = dynamic_cast<ConstValueNode *>(left.get());
        ConstValueNode *rightVal = dynamic_cast<ConstValueNode *>(right.get());
        if (leftVal && rightVal)
        {
            bool b(false);
            if (type == NodeType::Equal)
                b = (leftVal->value() == rightVal->value());
            else if (type == NodeType::NotEqual)
                b = (leftVal->value() != rightVal->value());
            else if (type == NodeType::Greater)
                b = (leftVal->value() > rightVal->value());
            else if (type == NodeType::GreaterEqual)
                b = (leftVal->value() >= rightVal->value());
            else if (type == NodeType::Less)
                b = (leftVal->value() < rightVal->value());
            else if (type == NodeType::LessEqual)
                b = (leftVal->value() <= rightVal->value());
            expr.pushNode(NodePtr(new ConstLogicalNode(b)));
        }
        else
        {
            if (left->isBool() || right->isBool())
            {
                setError("Can't apply '" + curToken().sval() + "' to "
                    "logical expression.");
                return false;
            }
            expr.pushNode(NodePtr(new CompareNode(type, std::move(left), std::move(right))));
        }
    }
    return true;
}

bool ConditionalParser::addexpr(Expression& expr)
{
    if (!multexpr(expr))
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

        if (!multexpr(expr))
        {
            setError("Expected expression following '" +
                curToken().sval() + "'.");
            return false;
        }

        NodePtr right = expr.popNode();
        NodePtr left = expr.popNode();

        ConstValueNode *leftVal = dynamic_cast<ConstValueNode *>(left.get());
        ConstValueNode *rightVal = dynamic_cast<ConstValueNode *>(right.get());
        if (leftVal && rightVal)
        {
            double v = (type == NodeType::Add) ?
                leftVal->value() + rightVal->value() :
                leftVal->value() - rightVal->value();
            expr.pushNode(NodePtr(new ConstValueNode(v)));
        }
        else
        {
            if (left->isBool() || right->isBool())
            {
                setError("Can't apply '" + curToken().sval() + "' to "
                    "logical expression.");
                return false;
            }
            expr.pushNode(NodePtr(new BinMathNode(type, std::move(left), std::move(right))));
        }
    }
    return true;
}

bool ConditionalParser::multexpr(Expression& expr)
{
    if (!notexpr(expr))
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

        if (!notexpr(expr))
        {
            setError("Expected expression following '" + curToken().sval() + "'.");
            return false;
        }

        NodePtr right = expr.popNode();
        NodePtr left = expr.popNode();

        ConstValueNode *leftVal = dynamic_cast<ConstValueNode *>(left.get());
        ConstValueNode *rightVal = dynamic_cast<ConstValueNode *>(right.get());

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
            expr.pushNode(NodePtr(new ConstValueNode(v)));
        }
        else
        {
            if (left->isBool() || right->isBool())
            {
                setError("Can't apply '" + curToken().sval() + "' to "
                    "logical expression.");
                return false;
            }
            expr.pushNode(NodePtr(new BinMathNode(type, std::move(left), std::move(right))));
        }
    }
    return true;
}

bool ConditionalParser::notexpr(Expression& expr)
{
    if (!match(TokenType::Not))
        return uminus(expr);

    if (!uminus(expr))
    {
        setError("Expected expression following '!'.");
        return false;
    }

    NodePtr sub = expr.popNode();
    if (sub->isValue())
    {
        setError("Can't apply '!' to numeric value.");
        return false;
    }
    expr.pushNode(NodePtr(new NotNode(NodeType::Not, std::move(sub))));
    return true;
}

bool ConditionalParser::uminus(Expression& expr)
{
    if (!match(TokenType::Dash))
        return primary(expr);

    if (!primary(expr))
    {
        setError("Expecting expression following '-'.");
        return false;
    }

    NodePtr sub = expr.popNode();
    ConstValueNode *node = dynamic_cast<ConstValueNode *>(sub.get());
    if (node)
    {
        double v = -(node->value());
        expr.pushNode(NodePtr(new ConstValueNode(v)));
    }
    else
    {
        if (node->isBool())
        {
            setError("Can't apply '-' to logical expression '" +
                sub->print() + "'.");
            return false;
        }
        expr.pushNode(NodePtr(new UnMathNode(NodeType::Negative, std::move(sub))));
    }
    return true;
}

bool ConditionalParser::primary(Expression& expr)
{
    if (match(TokenType::Number))
    {
        expr.pushNode(NodePtr(new ConstValueNode(curToken().dval())));
        return true;
    }
    else if (match(TokenType::Identifier))
    {
        expr.pushNode(NodePtr(new VarNode(curToken().sval())));
        return true;
    }

    // Check if this is the start of a paren-expression.
    if (!match(TokenType::Lparen))
    {
        setError("Expected identifier or value and instead found '" + peekToken().sval() + "'.");
        return false;
    }

    return parexpr(expr);
}

bool ConditionalParser::parexpr(Expression& expr)
{
    if (!expression(expr))
    {
        setError("Expected expression following '('.");
        return false;
    }

    if (!match(TokenType::Rparen))
    {
        setError("Expected ')' following expression at '" +
            curToken().sval() + "'.");
        return false;
    }
    return true;
}

} // namespace expr
} // namespace pdal
