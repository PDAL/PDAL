#pragma once

#include <stdint.h>
#include <array>
#include <string>
#include <vector>

#include <pdal/SpatialReference.hpp>
#include <pdal/util/Bounds.hpp>

namespace pdal
{
namespace ept
{

struct Options
{
    std::string outputDir;
    StringList inputFiles;
    std::string tempDir;
    bool doCube;
    size_t fileLimit;
    int level;
    int progressFd;
    StringList dimNames;
    bool stats;
};

struct BaseInfo
{
    BaseInfo(PointTableRef t) : table(t)
    {}

    pdal::BOX3D bounds;
    pdal::BOX3D trueBounds;
    size_t pointSize;
    std::string outputDir;
    int maxLevel;
    PointTableRef table;
    pdal::SpatialReference srs;

    using d3 = std::array<double, 3>;
    d3 scale { -1.0, -1.0, -1.0 };
    d3 offset {};
};

} // namespace ept
} // namespace pdal
