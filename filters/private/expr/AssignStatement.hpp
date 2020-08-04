#pragma once

#include "AssignParser.hpp"
#include "ConditionalExpression.hpp"
#include "IdentExpression.hpp"
#include "MathExpression.hpp"
#include "Lexer.hpp"

namespace pdal
{
namespace expr
{

class AssignStatement
{
public:
    AssignStatement() = default;
    AssignStatement(const AssignStatement&) = default;
    AssignStatement(AssignStatement&&) = default;
    AssignStatement& operator=(const AssignStatement&) = default;

    virtual ~AssignStatement()
    {}

    MathExpression& valueExpr();
    IdentExpression& identExpr();
    ConditionalExpression& conditionalExpr();
    bool valid() const;
    operator bool() const;

    std::string print() const;
    Utils::StatusWithReason prepare(PointLayoutPtr layout);

private:
    IdentExpression m_identExpr;
    MathExpression m_valueExpr;
    ConditionalExpression m_conditionalExpr;
    std::string m_error;
};

} // namespace expr

namespace Utils
{

template<>
inline StatusWithReason fromString(const std::string& from,
    pdal::expr::AssignStatement& stmt)
{
    expr::Lexer lexer(from);
    expr::AssignParser parser(lexer);
    bool ok = parser.statement(stmt) && parser.checkEnd();
    return { ok ? 0 : -1, parser.error() };
}

} // namespace Util

} // namespace pdal
