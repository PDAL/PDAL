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

#include <cstdlib>
#include <iostream>
#include <limits>

#include <json/json.h>

#include <entwine/types/dir.hpp>
#include <entwine/types/point.hpp>

namespace entwine
{

class Delta;

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
        return
            width() > 0 && depth() > 0 &&
            other.width() > 0 && other.depth() > 0 &&
            max().x > other.min().x && min().x < other.max().x &&
            max().y > other.min().y && min().y < other.max().y &&
            (force2d || (
                (!height() && !other.height()) || (
                    height() > 0 && other.height() > 0 &&
                    max().z > other.min().z && min().z < other.max().z)));
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
    bool contains(const Point& p) const
    {
        if (is3d()) return p >= m_min && p < m_max;
        else return
            p.x >= m_min.x && p.x < m_max.x &&
            p.y >= m_min.y && p.y < m_max.y;
    }

    double width()  const { return m_max.x - m_min.x; } // Length in X.
    double depth()  const { return m_max.y - m_min.y; } // Length in Y.
    double height() const { return m_max.z - m_min.z; } // Length in Z.
    explicit operator bool() const { return width() && depth() && height(); }

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

    Bounds getNw() const { return getNwd(true); }
    Bounds getNe() const { return getNed(true); }
    Bounds getSw() const { return getSwd(true); }
    Bounds getSe() const { return getSed(true); }

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

    bool empty() const { return !exists(); }
    bool exists() const { return min().exists() || max().exists(); }
    bool is3d() const { return m_min.z != m_max.z; }

    Json::Value toJson() const;
    Bounds to2d() const { return Bounds(min().x, min().y, max().x, max().y); }

    void grow(const Bounds& bounds);
    void grow(const Point& p);
    void shrink(const Bounds& bounds);

    bool isCubic() const
    {
        return width() == depth() && (!is3d() || width() == height());
    }

    // Bloat all coordinates necessary to form a cube and also to the nearest
    // integer.
    Bounds cubeify(const Delta* delta = nullptr) const;
    Bounds cubeify(const Delta& delta) const;

    Bounds deltify(const Delta* delta) const;
    Bounds deltify(const Delta& delta) const;
    Bounds undeltify(const Delta* delta) const;
    Bounds undeltify(const Delta& delta) const;

    Bounds growBy(double ratio) const;

    std::vector<Bounds> explode() const;
    std::vector<Bounds> explode(std::size_t delta) const;

    Bounds transform(const Transformation& t) const
    {
        return Bounds(Point::transform(min(), t), Point::transform(max(), t));
    }

    Bounds intersection(const Bounds& b) const
    {
        if (!this->overlaps(b)) return Bounds();
        return Bounds(Point::max(min(), b.min()), Point::min(max(), b.max()));
    }

    Bounds scale(const Point& scale, const Point& offset) const
    {
        return Bounds(
                Point::scale(min(), scale, offset),
                Point::scale(max(), scale, offset));
    }

    Bounds unscale(const Point& scale, const Point& offset) const
    {
        return Bounds(
                Point::unscale(min(), scale, offset),
                Point::unscale(max(), scale, offset));
    }

    static Bounds everything()
    {
        static const double dmin(std::numeric_limits<double>::lowest());
        static const double dmax(std::numeric_limits<double>::max());
        return Bounds(dmin, dmin, dmin, dmax, dmax, dmax);
    }

    static Bounds expander()
    {
        static const double dmin(std::numeric_limits<double>::lowest());
        static const double dmax(std::numeric_limits<double>::max());

        // Use Bounds::set to avoid malformed bounds warning.
        Bounds b;
        b.set(Point(dmax, dmax, dmax), Point(dmin, dmin, dmin));
        return b;
    }

    double operator[](std::size_t i) const
    {
        return i < 3 ? min()[i] : max()[i - 3];
    }

    Bounds make2d() const
    {
        return Bounds(min().x, min().y, max().x, max().y);
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

inline Bounds operator+(const Bounds& b, const Point& p)
{
    return Bounds(b.min() + p, b.max() + p);
}

} // namespace entwine

