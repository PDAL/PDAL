/******************************************************************************
* Copyright (c) 2016, Connor Manning (connor@hobu.co)
*
* Entwine -- Point cloud indexing
*
* Supporting libraries released under PDAL licensing by Hobu, inc.
*
******************************************************************************/

#pragma once

#include <cstdlib>
#include <iostream>

#include <json/json.h>

#include "dir.hpp"
#include "point.hpp"

namespace pdal
{
namespace entwine
{

class Bounds
{
public:
    Bounds() = default;
    Bounds(const Point& min, const Point& max);
    Bounds(const Point& center, double radius);
    Bounds(const Json::Value& json);
    Bounds(double xMin, double yMin, double xMax, double yMax);
    Bounds(
            double xMin,
            double yMin,
            double zMin,
            double xMax,
            double yMax,
            double zMax);

    void set(const Bounds& other)
    {
        m_min = other.m_min;
        m_max = other.m_max;
        m_mid = other.m_mid;
    }

    void set(const Point& min, const Point& max)
    {
        m_min = min;
        m_max = max;
        setMid();
    }

    const Point& min() const { return m_min; }
    const Point& max() const { return m_max; }
    const Point& mid() const { return m_mid; }

    // Returns true if these Bounds share any area in common with another.
    bool overlaps(const Bounds& other, bool force2d = false) const
    {
        Point otherMid(other.mid());

        return
            std::abs(m_mid.x - otherMid.x) <=
                width() / 2.0 + other.width() / 2.0 &&
            std::abs(m_mid.y - otherMid.y) <=
                depth() / 2.0 + other.depth() / 2.0 &&
            (force2d || std::abs(m_mid.z - otherMid.z) <=
                height() / 2.0 + other.height() / 2.0);
    }

    // Returns true if the requested Bounds are contained within these Bounds.
    bool contains(const Bounds& other, bool force2d = false) const
    {
        if (!force2d)
        {
            return m_min <= other.m_min && m_max >= other.m_max;
        }
        else
        {
            return
                m_min.x <= other.m_min.x &&
                m_min.y <= other.m_min.y &&
                m_max.x >= other.m_min.x &&
                m_max.y >= other.m_min.y;
        }
    }

    // Returns true if the requested point is contained within these Bounds.
    bool contains(const Point& p) const { return p >= m_min && p < m_max; }

    double width()  const { return m_max.x - m_min.x; } // Length in X.
    double depth()  const { return m_max.y - m_min.y; } // Length in Y.
    double height() const { return m_max.z - m_min.z; } // Length in Z.

    double area() const { return width() * depth(); }
    double volume() const { return width() * depth() * height(); }

    void goNwu()
    {
        m_max.x = m_mid.x;
        m_min.y = m_mid.y;
        m_min.z = m_mid.z;
        setMid();
    }

    void goNwd(const bool force2d = false)
    {
        m_max.x = m_mid.x;
        m_min.y = m_mid.y;
        if (!force2d) m_max.z = m_mid.z;
        setMid();
    }

    void goNeu()
    {
        m_min.x = m_mid.x;
        m_min.y = m_mid.y;
        m_min.z = m_mid.z;
        setMid();
    }

    void goNed(const bool force2d = false)
    {
        m_min.x = m_mid.x;
        m_min.y = m_mid.y;
        if (!force2d) m_max.z = m_mid.z;
        setMid();
    }

    void goSwu()
    {
        m_max.x = m_mid.x;
        m_max.y = m_mid.y;
        m_min.z = m_mid.z;
        setMid();
    }

    void goSwd(const bool force2d = false)
    {
        m_max.x = m_mid.x;
        m_max.y = m_mid.y;
        if (!force2d) m_max.z = m_mid.z;
        setMid();
    }

    void goSeu()
    {
        m_min.x = m_mid.x;
        m_max.y = m_mid.y;
        m_min.z = m_mid.z;
        setMid();
    }

    void goSed(const bool force2d = false)
    {
        m_min.x = m_mid.x;
        m_max.y = m_mid.y;
        if (!force2d) m_max.z = m_mid.z;
        setMid();
    }

    Bounds getNwd(bool force2d = false) const
    {
        Bounds b(*this); b.goNwd(force2d); return b;
    }

    Bounds getNed(bool force2d = false) const
    {
        Bounds b(*this); b.goNed(force2d); return b;
    }

    Bounds getSwd(bool force2d = false) const
    {
        Bounds b(*this); b.goSwd(force2d); return b;
    }

    Bounds getSed(bool force2d = false) const
    {
        Bounds b(*this); b.goSed(force2d); return b;
    }

    Bounds getNwu() const { Bounds b(*this); b.goNwu(); return b; }
    Bounds getNeu() const { Bounds b(*this); b.goNeu(); return b; }
    Bounds getSwu() const { Bounds b(*this); b.goSwu(); return b; }
    Bounds getSeu() const { Bounds b(*this); b.goSeu(); return b; }

    void go(Dir dir, bool force2d = false)
    {
        if (force2d) dir = toDir(toIntegral(dir, true));

        switch (dir)
        {
            case Dir::swd: goSwd(force2d); break;
            case Dir::sed: goSed(force2d); break;
            case Dir::nwd: goNwd(force2d); break;
            case Dir::ned: goNed(force2d); break;
            case Dir::swu: goSwu(); break;
            case Dir::seu: goSeu(); break;
            case Dir::nwu: goNwu(); break;
            case Dir::neu: goNeu(); break;
        }
    }

    Bounds get(Dir dir, bool force2d = false) const
    {
        if (force2d) dir = toDir(toIntegral(dir, true));

        switch (dir)
        {
            case Dir::swd: return getSwd(force2d); break;
            case Dir::sed: return getSed(force2d); break;
            case Dir::nwd: return getNwd(force2d); break;
            case Dir::ned: return getNed(force2d); break;
            case Dir::swu: return getSwu(); break;
            case Dir::seu: return getSeu(); break;
            case Dir::nwu: return getNwu(); break;
            case Dir::neu: return getNeu(); break;
        }

        throw std::runtime_error(
                "Invalid Dir to Bounds::get: " +
                std::to_string(toIntegral(dir)));
    }

    bool exists() const { return Point::exists(m_min) || Point::exists(m_max); }
    bool is3d() const { return m_min.z || m_max.z; }

    Json::Value toJson() const;

    void grow(const Bounds& bounds);
    void grow(const Point& p);
    void shrink(const Bounds& bounds);

    bool isCubic() const
    {
        return width() == depth() && (!is3d() || width() == height());
    }

    Bounds growBy(double ratio) const;

    std::vector<Bounds> explode() const;

    Bounds transform(const Transformation& t) const
    {
        return Bounds(Point::transform(min(), t), Point::transform(max(), t));
    }

    Bounds intersection(const Bounds& b) const
    {
        return Bounds(Point::max(min(), b.min()), Point::min(max(), b.max()));
    }

    Bounds scale(const Point& scale, const Point& offset)
    {
        return Bounds(
                Point::scale(min(), scale, offset),
                Point::scale(max(), scale, offset));
    }

    Bounds unscale(const Point& scale, const Point& offset)
    {
        return Bounds(
                Point::unscale(min(), scale, offset),
                Point::unscale(max(), scale, offset));
    }

private:
    Point m_min;
    Point m_max;
    Point m_mid;

    void setMid()
    {
        m_mid.x = m_min.x + (m_max.x - m_min.x) / 2.0;
        m_mid.y = m_min.y + (m_max.y - m_min.y) / 2.0;
        m_mid.z = m_min.z + (m_max.z - m_min.z) / 2.0;
    }
};

std::ostream& operator<<(std::ostream& os, const Bounds& bounds);

// Orders Bounds by their midpoint.  This is really only useful if the bounds
// are arranged in a grid and are of equal size (like during a MetaQuery).
inline bool operator<(const Bounds& lhs, const Bounds& rhs)
{
    const auto& lhsMid(lhs.mid());
    const auto& rhsMid(rhs.mid());

    return
        (lhsMid.x < rhsMid.x) ||
        (lhsMid.x == rhsMid.x && lhsMid.y < rhsMid.y) ||
        (lhsMid.x == rhsMid.x && lhsMid.y == rhsMid.y && lhsMid.z < rhsMid.z);
}

inline bool operator==(const Bounds& lhs, const Bounds& rhs)
{
    return lhs.min() == rhs.min() && lhs.max() == rhs.max();
}

inline bool operator!=(const Bounds& lhs, const Bounds& rhs)
{
    return !(lhs == rhs);
}

} // namespace entwine
} // namespace pdal

