/******************************************************************************
* Copyright (c) 2016, Connor Manning (connor@hobu.co)
*
* Entwine -- Point cloud indexing
*
* Supporting libraries released under PDAL licensing by Hobu, inc.
*
******************************************************************************/

#pragma once

#include <cmath>
#include <iomanip>
#include <limits>
#include <ostream>

namespace pdal { namespace greyhound {

class Point
{
public:
    Point() noexcept
        : x(Point::emptyCoord())
        , y(Point::emptyCoord())
        , z(Point::emptyCoord())
    { }

    Point(double x, double y) noexcept : x(x), y(y), z(Point::emptyCoord()) { }
    Point(double x, double y, double z) noexcept : x(x), y(y), z(z) { }

    double sqDist2d(const Point& other) const
    {
        const double xDelta(x - other.x);
        const double yDelta(y - other.y);

        return xDelta * xDelta + yDelta * yDelta;
    }

    // Calculates the distance-squared to another point.
    double sqDist3d(const Point& other) const
    {
        const double zDelta(z - other.z);
        return sqDist2d(other) + zDelta * zDelta;
    }

    static bool exists(Point p)
    {
        return
            p.x != Point::emptyCoord() ||
            p.y != Point::emptyCoord() ||
            p.z != Point::emptyCoord();
    }

    static bool exists(double x, double y, double z)
    {
        return exists(Point(x, y, z));
    }

    static double emptyCoord()
    {
        return 0;
    }

    double x;
    double y;
    double z;
};

inline bool operator<(const Point& lhs, const Point& rhs)
{
    return lhs.x < rhs.x && lhs.y < rhs.y && lhs.z < rhs.z;
}

inline bool operator<=(const Point& lhs, const Point& rhs)
{
    return lhs.x <= rhs.x && lhs.y <= rhs.y && lhs.z <= rhs.z;
}

inline bool operator>(const Point& lhs, const Point& rhs)
{
    return lhs.x > rhs.x && lhs.y > rhs.y && lhs.z > rhs.z;
}

inline bool operator>=(const Point& lhs, const Point& rhs)
{
    return lhs.x >= rhs.x && lhs.y >= rhs.y && lhs.z >= rhs.z;
}

inline bool operator==(const Point& lhs, const Point& rhs)
{
    return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z;
}

inline Point operator+(const Point& in, double offset)
{
    return Point(in.x + offset, in.y + offset, in.z + offset);
}

inline Point operator-(const Point& in, double offset)
{
    return in + (-offset);
}

inline std::ostream& operator<<(std::ostream& os, const Point& point)
{
    auto flags(os.flags());
    auto precision(os.precision());

    os << std::setprecision(2) << std::fixed;

    os << "(" << point.x << ", " << point.y;
    if (
            point.z != Point::emptyCoord() &&
            point.z != std::numeric_limits<double>::max() &&
            point.z != std::numeric_limits<double>::lowest())
    {
        os << ", " << point.z;
    }
    os << ")";

    os << std::setprecision(precision);
    os.flags(flags);

    return os;
}

} // namespace greyhound
} // namepsace pdal
