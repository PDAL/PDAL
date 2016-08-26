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

inline std::string dirToString(Dir dir)
{
    switch (dir)
    {
        case Dir::swd: return "swd"; break;
        case Dir::sed: return "sed"; break;
        case Dir::nwd: return "nwd"; break;
        case Dir::ned: return "ned"; break;
        case Dir::swu: return "swu"; break;
        case Dir::seu: return "seu"; break;
        case Dir::nwu: return "nwu"; break;
        case Dir::neu: return "neu"; break;
    }

    throw std::runtime_error("Cannot convert invalid Dir to string");
}

inline Dir stringToDir(const std::string& s)
{
    return static_cast<Dir>(
            (s[0] == 'n' ? 2 : 0) + // North? + 2.
            (s[1] == 'e' ? 1 : 0) + // East? +1.
            (s[2] == 'u' ? 4 : 0)); // Up? +4.
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
