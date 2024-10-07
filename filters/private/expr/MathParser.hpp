#pragma once

#include "BaseParser.hpp"

namespace pdal
{
namespace expr
{

class MathParser : public BaseParser
{
public:
    using BaseParser::BaseParser;

    bool expression(Expression& expr);

protected:
    bool valueexpr(Expression& expr);
    bool addexpr(Expression& expr);
    bool multexpr(Expression& expr);
    bool uminus(Expression& expr);
    bool primary(Expression& expr);
    bool parexpr(Expression& expr);
    bool function(Expression& expr);
    bool function0(Expression& expr);
    bool function1(Expression& expr);
};

} // namespace expr
} // namespace pdal
