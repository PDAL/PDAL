#include "MathParser.hpp"

namespace pdal
{
namespace expr
{

bool MathParser::expression(Expression& expr)
{
    return addexpr(expr);
}

bool MathParser::addexpr(Expression& expr)
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

bool MathParser::multexpr(Expression& expr)
{
    if (!uminus(expr))
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

        if (!uminus(expr))
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

bool MathParser::uminus(Expression& expr)
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

bool MathParser::primary(Expression& expr)
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

    return parexpr(expr);
}
    
bool MathParser::parexpr(Expression& expr)
{
    if (!match(TokenType::Lparen))
        return false;

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
