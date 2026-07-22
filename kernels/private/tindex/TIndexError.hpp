#pragma once

namespace pdal
{
namespace tindex
{

class TIndexError : public std::runtime_error
{
public:
    TIndexError(const std::string txt) : std::runtime_error(txt)
    {}
};

} // namespace tindex
} // namespace pdal

