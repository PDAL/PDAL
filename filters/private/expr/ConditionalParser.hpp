#pragma once

#include "MathParser.hpp"

namespace pdal
{
namespace expr
{

class ConditionalParser : public MathParser
{
public:
    ConditionalParser(Lexer& lexer) : MathParser(lexer)
    {}
    bool expression(Expression& expr) override;

protected:
    bool orexpr(Expression& expr);
    bool andexpr(Expression& expr);
    bool notexpr(Expression& expr);
    bool primarylogexpr(Expression& expr);
    bool compareexpr(Expression& expr);
    bool function1(Expression& expr);
};

} // namespace expr
} // namespace pdal
