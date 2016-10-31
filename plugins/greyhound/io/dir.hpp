/******************************************************************************
* Copyright (c) 2016, Connor Manning (connor@hobu.co)
*
* Entwine -- Point cloud indexing
*
* Supporting libraries released under PDAL licensing by Hobu, inc.
*
******************************************************************************/

#pragma once

#include "point.hpp"

namespace pdal { namespace greyhound {

enum class Dir
{
    swd = 0,
    sed = 1,
    nwd = 2,
    ned = 3,
    swu = 4,
    seu = 5,
    nwu = 6,
    neu = 7
};

// Get the direction of point P in relation to an origin O.
inline Dir getDirection(const Point& p, const Point& o)
{
    return static_cast<Dir>(
            (p.y >= o.y ? 2 : 0) +  // North? +2.
            (p.x >= o.x ? 1 : 0) +  // East? +1.
            (p.z >= o.z ? 4 : 0));  // Up? +4.
}

inline std::size_t toIntegral(Dir dir)
{
    return static_cast<std::size_t>(dir);
}

inline Dir toDir(std::size_t val)
{
    return static_cast<Dir>(val);
}

} // namespace entwine
} // namespace pdal
