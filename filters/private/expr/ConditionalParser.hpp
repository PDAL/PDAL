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
    bool notexpr(Expression& expr);
    bool orexpr(Expression& expr);
    bool andexpr(Expression& expr);
    bool compareexpr(Expression& expr);
    bool addexpr(Expression& expr);
    bool multexpr(Expression& expr);
    bool uminus(Expression& expr);
    bool primary(Expression& expr);
    bool parexpr(Expression& expr);
};

} // namespace expr
} // namespace pdal
