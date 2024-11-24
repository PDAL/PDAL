#include "ConditionalParser.hpp"
#include "MathParser.hpp"

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
    if (!notexpr(expr))
        return false;

    while (true)
    {
        if (!match(TokenType::And))
            return true;

        if (!notexpr(expr))
        {
            setError("Expected expression following '&&'.");
            return false;
        }

        NodePtr right = expr.popNode();
        NodePtr left = expr.popNode();

        if (left->isValue())
        {
            setError("Can't apply '&&' to numeric expression '" + left->print() + "'.");
            return false;
        }
        if (right->isValue())
        {
            setError("Can't apply '&&' to numeric expression '" + right->print() + "'.");
            return false;
        }
        expr.pushNode(NodePtr(new BoolNode(NodeType::And, std::move(left), std::move(right))));
    }
    return true;
}

bool ConditionalParser::notexpr(Expression& expr)
{
    if (!match(TokenType::Not))
        return primarylogexpr(expr);

    if (!primarylogexpr(expr))
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

bool ConditionalParser::primarylogexpr(Expression& expr)
{
    if (parexpr(expr))
        return true;

    if (function1(expr))
        return true;

    if (compareexpr(expr))
        return true;

    setError("Expected logical expression following '" + curToken().sval() + "'.");
    return false;
}

bool ConditionalParser::parexpr(Expression& expr)
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


//ABELL - This treats == and >= at the same precendence level.  In C++,
// <, >, <=, >= come before ==, !=
bool ConditionalParser::compareexpr(Expression& expr)
{
    MathParser mathparser(lexer());
    if (!mathparser.expression(expr))
    {
        setError(mathparser.error());
        return false;
    }

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

        if (!mathparser.expression(expr))
        {
            setError(mathparser.error());
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

bool ConditionalParser::function1(Expression& expr)
{
    auto checkMax = [](double d) -> bool
    {
        return d == std::numeric_limits<double>::max();
    };

    auto checkMin = [](double d) -> bool
    {
        return d == std::numeric_limits<double>::lowest();
    };

    static const std::vector<BoolFunc1> funcs {
        { "isnan", (BoolFunc1::Ptr)std::isnan },
        { "ismax", checkMax },
        { "ismin", checkMin }
    };

    std::string name = peekToken().sval();
    auto it = std::find_if(funcs.begin(), funcs.end(),
        [&name](const BoolFunc1& f){ return f.name == name; });

    if (it == funcs.end())
        return false;

    match(TokenType::Identifier);  // Move past identifier token. Guaranteed to work.

    if (!match(TokenType::Lparen))
    {
        setError("Expecting '(' to open function invocation of '" + name + "'.");
        return false;
    }

    MathParser mathparser(lexer());
    if (!mathparser.expression(expr))
    {
        setError(mathparser.error());
        return false;
    }

    if (!match(TokenType::Rparen))
    {
        setError("Expecting ')' following '" + name + "' argument.");
        return false;
    }

    NodePtr sub = expr.popNode();  // Pop the value expression.
    expr.pushNode(NodePtr(new BoolFuncNode(NodeType::Function, *it, std::move(sub))));
    return true;
}

} // namespace expr
} // namespace pdal
