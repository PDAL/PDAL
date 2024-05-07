#pragma once

#include "Expression.hpp"

namespace pdal
{
namespace expr
{

class IdentExpression : public Expression
{
public:
    Utils::StatusWithReason prepare(PointLayoutPtr layout);
    Dimension::Id eval() const;
    std::string name() const;
};

} // namespace expr
} // namespace pdal
