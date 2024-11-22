#pragma once

#include "BaseParser.hpp"

namespace pdal
{
namespace expr
{

class ConditionalParser : public BaseParser
{
public:
    ConditionalParser(Lexer& lexer) : BaseParser(lexer)
    {}
    bool expression(Expression& expr);

protected:
    bool orexpr(Expression& expr);
    bool andexpr(Expression& expr);
    bool notexpr(Expression& expr);
    bool primarylogexpr(Expression& expr);
    bool compareexpr(Expression& expr);
    bool parexpr(Expression& expr);
    bool function1(Expression& expr);
};

} // namespace expr
} // namespace pdal
