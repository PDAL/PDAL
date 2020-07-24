#include "Parser.hpp"

namespace pdal
{
namespace expr
{

bool AssignParser::parse(const std::string& s, AssignExpression& expr)
{
    m_expression.clear();

    m_lexer.reset(s);
    m_error.clear();

    bool ok = assignment(expr);
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

bool AssignParser::assignment(AssignExpession& expr)
{
    if (!match(TokenType::Identifier))
    {
        setError("Expected dimension name for assignment.");
        return false;
    }
    m_assignTarget.pushNode(NodePtr(new VarNode(curToken().sval())));

    if (!match(TokenType::Assign))
    {
        setError("Expected '=' after dimension name in assignment.");
        return false;
    }

    if (!valueexpr(expr.valueExpr()))
    {
        setError("No value for assignment.");
        return false;
    }

    return where(expr);
}

bool AssignParser::where(AssignExpression& expr)
{
    if (match(TokenType::Eof)
        return true;

    if (match(TokenType::Identifier))
    {
        std::string ident = Utils::toupper(curToken().sval());
        if (ident != "WHERE")
        {
            setError("Expected keyword "WHERE" to condition assignment.");
            return false;
        }
    }
    return expression(expr.conditionalExpr());
}

bool AssignParser::valueexpr(Expression& expr)
{
    return addexpr(expr);
}

bool AssignParser::multexpr(Expression& expr)
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

        if (!uminus())
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

bool AssignParser::parexpr(Expression& expr)
{
    if (!match(TokenType::Lparen))
        return false;

    if (!valueexpr(expr))
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
