#include "AssignParser.hpp"
#include "AssignExpression.hpp"
#include "ConditionalParser.hpp"
#include "MathParser.hpp"

namespace pdal
{
namespace expr
{

bool AssignParser::expression(AssignExpression& expr)
{
    return assignment(expr);
}

bool AssignParser::assignment(AssignExpression& expr)
{
    if (!match(TokenType::Identifier))
    {
        setError("Expected dimension name for assignment.");
        return false;
    }
    expr.identExpr().pushNode(NodePtr(new VarNode(curToken().sval())));

    if (!match(TokenType::Assign))
    {
        setError("Expected '=' after dimension name in assignment.");
        return false;
    }

    MathParser parser(lexer());
    if (!parser.expression(expr.valueExpr()))
    {
        setError("No value for assignment.");
        return false;
    }

    return where(expr);
}

bool AssignParser::where(AssignExpression& expr)
{
    if (match(TokenType::Eof))
        return true;

    if (match(TokenType::Identifier))
    {
        std::string ident = Utils::toupper(curToken().sval());
        if (ident != "WHERE")
        {
            setError("Expected keyword 'WHERE' to precede condition assignment.");
            return false;
        }
    }
    ConditionalParser parser(lexer());
    return parser.expression(expr.conditionalExpr());
}

} // namespace expr
} // namespace pdal
