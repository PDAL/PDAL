#pragma once

#include "BaseParser.hpp"

namespace pdal
{
namespace expr
{

class AssignExpression;

class AssignParser : public BaseParser
{
public:
    AssignParser(Lexer& lexer) : BaseParser(lexer)
    {}

    bool expression(AssignExpression& expr);

protected:
    bool assignment(AssignExpression& expr);
    bool where(AssignExpression& expr);
};

} // namespace expr
} // namespace pdal
