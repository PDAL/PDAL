#pragma once

#include "BaseParser.hpp"

namespace pdal
{
namespace expr
{

class MathParser : public BaseParser
{
public:
    MathParser(Lexer& lexer) : BaseParser(lexer)
    {}

    bool expression(Expression& expr);

protected:
    bool valueexpr(Expression& expr);
    bool addexpr(Expression& expr);
    bool multexpr(Expression& expr);
    bool uminus(Expression& expr);
    bool primary(Expression& expr);
    bool parexpr(Expression& expr);
};

} // namespace expr
} // namespace pdal
