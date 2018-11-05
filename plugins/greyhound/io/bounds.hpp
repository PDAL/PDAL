/******************************************************************************
* Copyright (c) 2018, Connor Manning (connor@hobu.co)
*
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following
* conditions are met:
*
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in
*       the documentation and/or other materials provided
*       with the distribution.
*     * Neither the name of Hobu, Inc. or Flaxen Geo Consulting nor the
*       names of its contributors may be used to endorse or promote
*       products derived from this software without specific prior
*       written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
* OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
* AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
* OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
* OF SUCH DAMAGE.
****************************************************************************/

// This file was originally in Entwine, which is LGPL2, but it has been
// relicensed for inclusion in PDAL.


#pragma once

#include <cstdlib>
#include <iostream>
#include <limits>

#include <json/json.h>

#include "dir.hpp"
#include "point.hpp"

namespace pdal
{
namespace entwine
{

class PDAL_DLL Bounds
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

    const Point& minimum() const { return m_min; }
    const Point& maximum() const { return m_max; }
    const Point& mid() const { return m_mid; }

    // Returns true if these Bounds share any area in common with another.
    bool overlaps(const Bounds& other, bool force2d = false) const
    {
        return
            width() > 0 && depth() > 0 &&
            other.width() > 0 && other.depth() > 0 &&
            maximum().x > other.minimum().x &&
            minimum().x < other.maximum().x &&
            maximum().y > other.minimum().y &&
            minimum().y < other.maximum().y &&
            (force2d || (
                (!height() && !other.height()) || (
                    height() > 0 && other.height() > 0 &&
                    maximum().z > other.minimum().z &&
                    minimum().z < other.maximum().z)));
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
    bool exists() const { return minimum().exists() || maximum().exists(); }
    bool is3d() const { return m_min.z != m_max.z; }

    Json::Value toJson() const;
    Bounds to2d() const { return Bounds(minimum().x, minimum().y,
        maximum().x, maximum().y); }

    void grow(const Bounds& bounds);
    void grow(const Point& p);
    void shrink(const Bounds& bounds);

    bool isCubic() const
    {
        return width() == depth() && (!is3d() || width() == height());
    }

    Bounds growBy(double ratio) const;

    std::vector<Bounds> explode() const;
    std::vector<Bounds> explode(std::size_t delta) const;

    Bounds transform(const Transformation& t) const
    {
        return Bounds(Point::transform(minimum(), t),
            Point::transform(maximum(), t));
    }

    Bounds intersection(const Bounds& b) const
    {
        if (!this->overlaps(b)) return Bounds();
        return Bounds(Point::maximum(minimum(), b.minimum()),
            Point::minimum(maximum(), b.maximum()));
    }

    Bounds scale(const Point& scale, const Point& offset) const
    {
        return Bounds(
                Point::scale(minimum(), scale, offset),
                Point::scale(maximum(), scale, offset));
    }

    Bounds unscale(const Point& scale, const Point& offset) const
    {
        return Bounds(
                Point::unscale(minimum(), scale, offset),
                Point::unscale(maximum(), scale, offset));
    }

    static Bounds everything()
    {
        static const double dmin(std::numeric_limits<double>::lowest());
        static const double dmax((std::numeric_limits<double>::max)());
        return Bounds(dmin, dmin, dmin, dmax, dmax, dmax);
    }

    static Bounds expander()
    {
        static const double dmin(std::numeric_limits<double>::lowest());
        static const double dmax((std::numeric_limits<double>::max)());

        // Use Bounds::set to avoid malformed bounds warning.
        Bounds b;
        b.set(Point(dmax, dmax, dmax), Point(dmin, dmin, dmin));
        return b;
    }

    double operator[](std::size_t i) const
    {
        return i < 3 ? minimum()[i] : maximum()[i - 3];
    }

    Bounds make2d() const
    {
        return Bounds(minimum().x, minimum().y, maximum().x, maximum().y);
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

PDAL_DLL std::ostream& operator<<(std::ostream& os, const Bounds& bounds);

// Orders Bounds by their midpoint.  This is really only useful if the bounds
// are arranged in a grid and are of equal size (like during a MetaQuery).
PDAL_DLL inline bool operator<(const Bounds& lhs, const Bounds& rhs)
{
    const auto& lhsMid(lhs.mid());
    const auto& rhsMid(rhs.mid());

    return
        (lhsMid.x < rhsMid.x) ||
        (lhsMid.x == rhsMid.x && lhsMid.y < rhsMid.y) ||
        (lhsMid.x == rhsMid.x && lhsMid.y == rhsMid.y && lhsMid.z < rhsMid.z);
}

PDAL_DLL inline bool operator==(const Bounds& lhs, const Bounds& rhs)
{
    return lhs.minimum() == rhs.minimum() && lhs.maximum() == rhs.maximum();
}

PDAL_DLL inline bool operator!=(const Bounds& lhs, const Bounds& rhs)
{
    return !(lhs == rhs);
}

PDAL_DLL inline Bounds operator+(const Bounds& b, const Point& p)
{
    return Bounds(b.minimum() + p, b.maximum() + p);
}

} // namespace entwine
} // namespace pdal

