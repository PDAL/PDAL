#include "AssignParser.hpp"
#include "AssignStatement.hpp"
#include "ConditionalParser.hpp"
#include "MathParser.hpp"

namespace pdal
{
namespace expr
{

bool AssignParser::statement(AssignStatement& expr)
{
    return assignment(expr);
}

bool AssignParser::assignment(AssignStatement& expr)
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

    static const std::vector<Token> endTokens
    {
        Token(TokenType::Eof),
        Token(TokenType::Identifier, "WHERE")
    };

    MathParser parser(lexer(), endTokens);
    if (!parser.expression(expr.valueExpr()))
    {
        setError(parser.error());
        return false;
    }
    if (!parser.checkEnd())
    {
        setError("Invalid token '" + peekToken().sval() + "' following valid math expression");
        return false;
    }

    return where(expr);
}

bool AssignParser::where(AssignStatement& expr)
{
    if (match(TokenType::Eof))
        return true;

    if (match(TokenType::Identifier))
    {
        std::string ident = Utils::toupper(curToken().sval());
        if (ident == "WHERE")
        {
            ConditionalParser parser(lexer());
            bool status = parser.expression(expr.conditionalExpr());
            if (!status)
                setError(parser.error());
            return status;
        }
    }

    setError("Expected keyword 'WHERE' to precede condition assignment. Found '" +
        peekToken().sval() + "' instead.");
    return false;
}

} // namespace expr
} // namespace pdal
