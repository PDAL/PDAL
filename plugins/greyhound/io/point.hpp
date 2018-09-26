/******************************************************************************
* Copyright (c) 2016, Connor Manning (connor@hobu.co)
*
* Entwine -- Point cloud indexing
*
* Entwine is available under the terms of the LGPL2 license. See COPYING
* for specific license text and more information.
*
******************************************************************************/

#pragma once

#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <vector>
#include <algorithm>

#include <json/json.h>
#include <pdal/pdal_export.hpp>

namespace pdal
{
namespace entwine
{

using Transformation = std::vector<double>;

class PDAL_DLL Point
{
public:
    Point() noexcept
        : x(Point::emptyCoord())
        , y(Point::emptyCoord())
        , z(Point::emptyCoord())
    { }

    Point(double v) noexcept : x(v), y(v), z(v) { }
    Point(double x, double y) noexcept : x(x), y(y), z(Point::emptyCoord()) { }
    Point(double x, double y, double z) noexcept : x(x), y(y), z(z) { }

    Point(const Json::Value& json)
        : Point()
    {
        if (!json.isNull())
        {
            if (json.isArray())
            {
                x = json[0].asDouble();
                y = json[1].asDouble();
                if (json.size() > 2) z = json[2].asDouble();
            }
            else if (json.isNumeric())
            {
                x = y = z = json.asDouble();
            }
            else if (json.isObject())
            {
                x = json["x"].asDouble();
                y = json["y"].asDouble();
                z = json["z"].asDouble();
            }
        }
    }

    Json::Value toJson() const { return toJsonArray(); }

    Json::Value toJsonArray() const
    {
        Json::Value json;
        json.append(x);
        json.append(y);
        json.append(z);
        return json;
    }

    Json::Value toJsonObject() const
    {
        Json::Value json;
        json["x"] = x;
        json["y"] = y;
        json["z"] = z;
        return json;
    }

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

    bool exists() const
    {
        return
            x != Point::emptyCoord() ||
            y != Point::emptyCoord() ||
            z != Point::emptyCoord();
    }

    static bool exists(double x, double y, double z)
    {
        return
            x != Point::emptyCoord() ||
            y != Point::emptyCoord() ||
            z != Point::emptyCoord();
    }

    static double emptyCoord()
    {
        return 0;
    }

    static Point maximum(const Point& a, const Point& b)
    {
        return Point(
                (std::max)(a.x, b.x),
                (std::max)(a.y, b.y),
                (std::max)(a.z, b.z));
    }

    static Point minimum(const Point& a, const Point& b)
    {
        return Point(
                (std::min)(a.x, b.x),
                (std::min)(a.y, b.y),
                (std::min)(a.z, b.z));
    }

    static Point normalize(const Point& p)
    {
        const double m(std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z));
        return Point(p.x / m, p.y / m, p.z / m);
    }

    static Point cross(const Point& a, const Point& b)
    {
        return Point(
                a.y * b.z - a.z * b.y,
                a.z * b.x - a.x * b.z,
                a.x * b.y - a.y * b.x);
    }

    static double dot(const Point& a, const Point& b)
    {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    static Point transform(const Point& p, const Transformation& t)
    {
        return Point(
            p.x * t[0] + p.y * t[1] + p.z * t[2] + t[3],
            p.x * t[4] + p.y * t[5] + p.z * t[6] + t[7],
            p.x * t[8] + p.y * t[9] + p.z * t[10] + t[11]);
    }

    static Point scale(const Point& p, const Point& scale, const Point& offset)
    {
        return Point(
                Point::scale(p.x, scale.x, offset.x),
                Point::scale(p.y, scale.y, offset.y),
                Point::scale(p.z, scale.z, offset.z));
    }

    static double scale(double d, double scale, double offset)
    {
        return (d - offset) / scale;
    }

    static Point scale(
            const Point& p,
            const Point& origin,
            const Point& scale,
            const Point& offset)
    {
        return Point(
                Point::scale(p.x, origin.x, scale.x, offset.x),
                Point::scale(p.y, origin.y, scale.y, offset.y),
                Point::scale(p.z, origin.z, scale.z, offset.z));
    }

    static double scale(double d, double origin, double scale, double offset)
    {
        return (d - origin) / scale + origin - offset;
    }

    static double scaleInversed(
            double d,
            double origin,
            double scaleInversed,
            double offset)
    {
        return (d - origin) * scaleInversed + origin - offset;
    }

    static Point unscale(
            const Point& p,
            const Point& scale,
            const Point& offset)
    {
        return Point(
                Point::unscale(p.x, scale.x, offset.x),
                Point::unscale(p.y, scale.y, offset.y),
                Point::unscale(p.z, scale.z, offset.z));
    }

    static double unscale(double d, double scale, double offset)
    {
        return d * scale + offset;
    }

    static Point unscale(
            const Point& p,
            const Point& origin,
            const Point& scale,
            const Point& offset)
    {
        return Point(
                Point::unscale(p.x, origin.x, scale.x, offset.x),
                Point::unscale(p.y, origin.y, scale.y, offset.y),
                Point::unscale(p.z, origin.z, scale.z, offset.z));
    }

    static double unscale(double d, double origin, double scale, double offset)
    {
        return (d - origin + offset) * scale + origin;
    }

    template<typename Op> Point apply(Op op) const
    {
        return Point(op(x), op(y), op(z));
    }

    template<typename Op> static Point apply(Op op, const Point& p)
    {
        return Point(op(p.x), op(p.y), op(p.z));
    }

    double& operator[](std::size_t i)
    {
        switch (i)
        {
            case 0: return x;
            case 1: return y;
            case 2: return z;
            default: throw std::runtime_error("Invalid coordinate index");
        }
    }

    double operator[](std::size_t i) const
    {
        switch (i)
        {
            case 0: return x;
            case 1: return y;
            case 2: return z;
            default: throw std::runtime_error("Invalid coordinate index");
        }
    }

    static Point round(const Point& p)
    {
        return Point(std::round(p.x), std::round(p.y), std::round(p.z));
    }

    double x;
    double y;
    double z;
};

inline bool ltChained(const Point& lhs, const Point& rhs)
{
    return
        (lhs.x < rhs.x) ||
        (lhs.x == rhs.x &&
            (lhs.y < rhs.y ||
            (lhs.y == rhs.y &&
                (lhs.z < rhs.z))));
}

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

inline bool operator!=(const Point& lhs, const Point& rhs)
{
    return !(lhs == rhs);
}

inline Point operator+(const Point& in, double offset)
{
    return Point(in.x + offset, in.y + offset, in.z + offset);
}

inline Point operator-(const Point& p)
{
    return Point(-p.x, -p.y, -p.z);
}

inline Point operator-(const Point& in, double offset)
{
    return in + (-offset);
}

inline Point operator+(const Point& lhs, const Point& rhs)
{
    return Point(lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z);
}

inline Point operator-(const Point& lhs, const Point& rhs)
{
    return Point(lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z);
}

inline Point& operator+=(Point& lhs, const Point& rhs)
{
    lhs.x += rhs.x;
    lhs.y += rhs.y;
    lhs.z += rhs.z;
    return lhs;
}

inline Point& operator-=(Point& lhs, const Point& rhs)
{
    lhs.x -= rhs.x;
    lhs.y -= rhs.y;
    lhs.z -= rhs.z;
    return lhs;
}

inline Point operator*(const Point& p, double s)
{
    return Point(p.x * s, p.y * s, p.z * s);
}

inline Point operator*(const Point& a, const Point& b)
{
    return Point(a.x * b.x, a.y * b.y, a.z * b.z);
}

inline Point operator/(const Point& a, double s)
{
    return Point(a.x / s, a.y / s, a.z / s);
}

inline Point operator/(const Point& a, const Point& b)
{
    return Point(a.x / b.x, a.y / b.y, a.z / b.z);
}

inline std::ostream& operator<<(std::ostream& os, const Point& point)
{
    auto flags(os.flags());
    auto precision(os.precision());

    auto printCoord([&os](double d)
    {
        if (d == (std::numeric_limits<double>::max)()) os << "max";
        else if (d == std::numeric_limits<double>::lowest()) os << "min";
        else if (std::trunc(d) == d) os << static_cast<long>(d);
        else
        {
            std::ostringstream oss;
            oss << std::setprecision(8) << d;
            std::string s(oss.str());

            // Don't strip out trailing zeroes after exponential notation.
            if (s.find_first_of("Ee") == std::string::npos)
            {
                while (
                        s.find('.') != std::string::npos &&
                        s.size() > 2 &&
                        s.back() == '0' &&
                        s[s.size() - 2] != '.')
                {
                    s.pop_back();
                }
            }

            os << s;
        }
    });

    os << std::setprecision(5) << std::fixed;

    os << "(";
    printCoord(point.x);
    os << ", ";
    printCoord(point.y);
    os << ", ";
    printCoord(point.z);
    os << ")";

    os << std::setprecision(precision);
    os.flags(flags);

    return os;
}

class Color
{
public:
    Color() : r(0), g(0), b(0) { }
    Color(uint8_t r, uint8_t g, uint8_t b) : r(r), g(g), b(b) { }

    static Color minimum(const Color& a, const Color& b)
    {
        return Color(
                (std::min)(a.r, b.r),
                (std::min)(a.g, b.g),
                (std::min)(a.b, b.b));
    }

    static Color maximum(const Color& a, const Color& b)
    {
        return Color(
                (std::max)(a.r, b.r),
                (std::max)(a.g, b.g),
                (std::max)(a.b, b.b));
    }

    uint8_t r, g, b;
};

inline std::ostream& operator<<(std::ostream& os, const Color& c)
{
    os << "(" << (int)c.r << ", " << (int)c.g << ", " << (int)c.b << ")";
    return os;
}

} // namespace entwine
} // namespace pdal

