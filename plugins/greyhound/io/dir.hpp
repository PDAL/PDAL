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

namespace pdal
{
namespace entwine
{

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

inline constexpr std::size_t dirHalfEnd() { return 4; }
inline constexpr std::size_t dirEnd() { return 8; }

// Get the direction from an origin O to a point P.
inline Dir getDirection(const Point& o, const Point& p)
{
    return static_cast<Dir>(
            (p.y >= o.y ? 2 : 0) +  // North? +2.
            (p.x >= o.x ? 1 : 0) +  // East? +1.
            (p.z >= o.z ? 4 : 0));  // Up? +4.
}

inline Dir getDirection(const Point& o, const Point& p, bool force2d)
{
    if (force2d)
    {
        return static_cast<Dir>(
                (p.y >= o.y ? 2 : 0) +  // North? +2.
                (p.x >= o.x ? 1 : 0));  // East? +1.
    }
    else
    {
        return getDirection(o, p);
    }
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

inline std::size_t toIntegral(Dir dir, bool force2d = false)
{
    std::size_t result(static_cast<std::size_t>(dir));
    if (force2d) result %= 4;
    return result;
}

inline Dir toDir(std::size_t val)
{
    return static_cast<Dir>(val);
}

inline bool isSouth(Dir dir) { return toIntegral(dir) % 4 < 2; } // 0, 1, 4, 5
inline bool isNorth(Dir dir) { return !isSouth(dir); }

inline bool isWest(Dir dir) { return toIntegral(dir) % 2 == 0; } // 0, 2, 4, 6
inline bool isEast(Dir dir) { return !isWest(dir); }

inline bool isDown(Dir dir) { return toIntegral(dir) < 4; }      // 0, 1, 2, 3
inline bool isUp(Dir dir) { return !isDown(dir); }

inline std::ostream& operator<<(std::ostream& os, Dir dir)
{
    return os << dirToString(dir);
}

} // namespace entwine
} // namespace pdal

