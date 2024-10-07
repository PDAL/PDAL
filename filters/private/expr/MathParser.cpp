#include <cmath>
#include <vector>

#include "MathParser.hpp"

namespace pdal
{
namespace expr
{

bool MathParser::expression(Expression& expr)
{
    // Only success if we've properly parsed the expression AND we're add the end of
    // a math expression.
    return addexpr(expr) && checkEnd();
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
            setError("Expected expression following '" + curToken().sval() + "'.");
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

    NodePtr sub = expr.popNode();  // Pop the primary.
    assert(sub.get());
    ConstValueNode *constnode = dynamic_cast<ConstValueNode *>(sub.get());
    if (constnode)
    {
        double v = -(constnode->value());
        expr.pushNode(NodePtr(new ConstValueNode(v)));
    }
    else
        expr.pushNode(NodePtr(new UnMathNode(NodeType::Negative, std::move(sub))));
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
        if (!function(expr))
            expr.pushNode(NodePtr(new VarNode(curToken().sval())));
        return true;
    }
    bool status = parexpr(expr);
    if (!status)
        setError("Expecting value expression, instead found '" + peekToken().sval() + "'.");

    return status;
}

bool MathParser::function(Expression& expr)
{
    if (function0(expr))
        return true;
    return function1(expr);
}

bool MathParser::function0(Expression& expr)
{
    struct Func0
    {
        std::string name;
        double value;
    };

    static const std::vector<Func0> funcs {
        { "nan", std::numeric_limits<double>::quiet_NaN() },
        { "lowest", std::numeric_limits<double>::lowest() },
        { "highest", (std::numeric_limits<double>::max)() }
    };

    std::string name = curToken().sval();
    auto it = std::find_if(funcs.begin(), funcs.end(),
        [&name](const Func0& f){ return f.name == name; });

    if (it == funcs.end())
    {
        if (peekToken() == TokenType::Lparen)
            setError("Invalid function name '" + name + "'");
        return false;
    }

    if (!match(TokenType::Lparen))
    {
        setError("Expecting '(' to open function invocation of '" + name + "'.");
        return false;
    }

    if (!match(TokenType::Rparen))
    {
        setError("Expecting ')' to close function invocation of '" + name + "'.");
        return false;
    }

    NodePtr sub = expr.popNode();  // Pop the primary.
    expr.pushNode(NodePtr(new ConstValueNode(it->value)));

    return true;
}

bool MathParser::function1(Expression& expr)
{
    static const std::vector<Func1> funcs {
        { "floor", ::floor },
        { "ceil", ::ceil },
        { "round", ::round },
        { "abs", ::fabs },
        { "fabs", ::fabs },
        { "sqrt", ::sqrt },
        { "sin", ::sin },
        { "cos", ::cos },
        { "tan", ::tan },
        { "asin", ::asin },
        { "acos", ::acos },
        { "atan", ::atan },
        { "sinh", ::sinh },
        { "cosh", ::cosh },
        { "tanh", ::tanh },
        { "asinh", ::asinh },
        { "acosh", ::acosh },
        { "log", ::log },
        { "log2", ::log2 },
        { "log10", ::log10 },
        { "exp", ::exp },
        { "exp2", ::exp2 }
    };

    std::string name = curToken().sval();
    auto it = std::find_if(funcs.begin(), funcs.end(),
        [&name](const Func1& f){ return f.name == name; });

    if (it == funcs.end())
    {
        if (peekToken() == TokenType::Lparen)
            setError("Invalid function name '" + name + "'");
        return false;
    }

    if (!match(TokenType::Lparen))
    {
        setError("Expecting '(' to open function invocation of '" + name + "'.");
        return false;
    }

    if (!addexpr(expr))
    {
        setError("Expecting expression following '" + name + "('.");
        return false;
    }

    if (!match(TokenType::Rparen))
    {
        setError("Expecting ')' following '" + name + "' argument.");
        return false;
    }

    NodePtr sub = expr.popNode();  // Pop the primary.
    expr.pushNode(NodePtr(new FuncNode(NodeType::Function, *it, std::move(sub))));

    return true;
}

bool MathParser::parexpr(Expression& expr)
{
    if (!match(TokenType::Lparen))
        return false;

    if (!addexpr(expr))
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
