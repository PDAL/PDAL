#pragma once

#include "BaseParser.hpp"

namespace pdal
{
namespace expr
{

class AssignStatement;

class AssignParser : public BaseParser
{
public:
    AssignParser(Lexer& lexer) : BaseParser(lexer)
    {}

    bool statement(AssignStatement& expr);

protected:
    bool assignment(AssignStatement& expr);
    bool where(AssignStatement& expr);
};

} // namespace expr
} // namespace pdal
