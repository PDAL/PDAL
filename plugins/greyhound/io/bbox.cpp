/******************************************************************************
* Copyright (c) 2016, Connor Manning (connor@hobu.co)
*
* Entwine -- Point cloud indexing
*
* Supporting libraries released under PDAL licensing by Hobu, inc.
*
******************************************************************************/

#include "bbox.hpp"

#include <cmath>
#include <limits>
#include <numeric>
#include <iostream>

#include "range.hpp"
#include "point.hpp"

namespace pdal { namespace greyhound {

BBox::BBox() : m_min(), m_max(), m_mid(), m_is3d(false) { }

BBox::BBox(const Point min, const Point max, const bool is3d)
    : m_min(
            std::min(min.x, max.x),
            std::min(min.y, max.y),
            std::min(min.z, max.z))
    , m_max(
            std::max(min.x, max.x),
            std::max(min.y, max.y),
            std::max(min.z, max.z))
    , m_mid()
    , m_is3d(is3d)
{
    setMid();
    check(min, max);
}

BBox::BBox(const BBox& other)
    : m_min(other.min())
    , m_max(other.max())
    , m_mid(other.m_mid)
    , m_is3d(other.m_is3d)
{ }

BBox::BBox(const Json::Value& json)
    : m_min()
    , m_max()
    , m_mid()
    , m_is3d(true)
{
    if (!json.isArray() || (json.size() != 4 && json.size() != 6))
    {
        std::string what(
                "Invalid JSON BBox specification: " + json.toStyledString());
        throw std::runtime_error(what);
    }

    m_is3d = (json.size() == 6);

    if (m_is3d)
    {
        m_min = Point(
                json.get(Json::ArrayIndex(0), 0).asDouble(),
                json.get(Json::ArrayIndex(1), 0).asDouble(),
                json.get(Json::ArrayIndex(2), 0).asDouble());
        m_max = Point(
                json.get(Json::ArrayIndex(3), 0).asDouble(),
                json.get(Json::ArrayIndex(4), 0).asDouble(),
                json.get(Json::ArrayIndex(5), 0).asDouble());
    }
    else
    {
        m_min = Point(
                json.get(Json::ArrayIndex(0), 0).asDouble(),
                json.get(Json::ArrayIndex(1), 0).asDouble());
        m_max = Point(
                json.get(Json::ArrayIndex(2), 0).asDouble(),
                json.get(Json::ArrayIndex(3), 0).asDouble());
    }

    check(m_min, m_max);
    setMid();
}

void BBox::set(const BBox& other)
{
    m_min = other.m_min;
    m_max = other.m_max;
    m_mid = other.m_mid;
    m_is3d = other.m_is3d;
}

void BBox::set(const Point& min, const Point& max, const bool is3d)
{
    m_min = min;
    m_max = max;
    m_is3d = is3d;
    setMid();
}

bool BBox::overlaps(const BBox& other) const
{
    Point otherMid(other.mid());

    return
        std::abs(m_mid.x - otherMid.x) <
            width() / 2.0  + other.width() / 2.0 &&
        std::abs(m_mid.y - otherMid.y) <
            depth() / 2.0 + other.depth() / 2.0 &&
        (
            !m_is3d ||
            !other.m_is3d ||
            std::abs(m_mid.z - otherMid.z) <
                height() / 2.0 + other.height() / 2.0);
}

bool BBox::contains(const BBox& other, const bool force2d) const
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

bool BBox::contains(const Point& p) const
{
    return
        p.x >= m_min.x && p.x < m_max.x &&
        p.y >= m_min.y && p.y < m_max.y &&
        (!m_is3d || (p.z >= m_min.z && p.z < m_max.z));

}

void BBox::goNwu()
{
    m_max.x = m_mid.x;
    m_min.y = m_mid.y;
    if (m_is3d) m_min.z = m_mid.z;
    setMid();
}

void BBox::goNwd(const bool force2d)
{
    m_max.x = m_mid.x;
    m_min.y = m_mid.y;
    if (!force2d && m_is3d) m_max.z = m_mid.z;
    setMid();
}

void BBox::goNeu()
{
    m_min.x = m_mid.x;
    m_min.y = m_mid.y;
    if (m_is3d) m_min.z = m_mid.z;
    setMid();
}

void BBox::goNed(const bool force2d)
{
    m_min.x = m_mid.x;
    m_min.y = m_mid.y;
    if (!force2d && m_is3d) m_max.z = m_mid.z;
    setMid();
}

void BBox::goSwu()
{
    m_max.x = m_mid.x;
    m_max.y = m_mid.y;
    if (m_is3d) m_min.z = m_mid.z;
    setMid();
}

void BBox::goSwd(const bool force2d)
{
    m_max.x = m_mid.x;
    m_max.y = m_mid.y;
    if (!force2d && m_is3d) m_max.z = m_mid.z;
    setMid();
}

void BBox::goSeu()
{
    m_min.x = m_mid.x;
    m_max.y = m_mid.y;
    if (m_is3d) m_min.z = m_mid.z;
    setMid();
}

void BBox::goSed(const bool force2d)
{
    m_min.x = m_mid.x;
    m_max.y = m_mid.y;
    if (!force2d && m_is3d) m_max.z = m_mid.z;
    setMid();
}

Json::Value BBox::toJson() const
{
    Json::Value json;

    json.append(m_min.x);
    json.append(m_min.y);
    if (m_is3d) json.append(m_min.z);

    json.append(m_max.x);
    json.append(m_max.y);
    if (m_is3d) json.append(m_max.z);

    return json;
}

void BBox::check(const Point& min, const Point& max) const
{
    if (min.x > max.x || min.y > max.y || min.z > max.z)
    {
        std::cout << "Correcting malformed BBox" << std::endl;
    }
}

void BBox::setMid()
{
    m_mid.x = m_min.x + (m_max.x - m_min.x) / 2.0;
    m_mid.y = m_min.y + (m_max.y - m_min.y) / 2.0;
    /* if (m_is3d) */ m_mid.z = m_min.z + (m_max.z - m_min.z) / 2.0;
}

void BBox::grow(const BBox& other)
{
    grow(other.min());
    grow(other.max());
}

void BBox::grow(const Point& p)
{
    m_min.x = std::min(m_min.x, p.x);
    m_min.y = std::min(m_min.y, p.y);
    m_min.z = std::min(m_min.z, p.z);
    m_max.x = std::max(m_max.x, p.x);
    m_max.y = std::max(m_max.y, p.y);
    m_max.z = std::max(m_max.z, p.z);
    setMid();
}

void BBox::growZ(const Range& range)
{
    m_min.z = std::min(m_min.z, range.min);
    m_max.z = std::max(m_max.z, range.max);
    m_mid.z = m_min.z + (m_max.z - m_min.z) / 2.0;
}

void BBox::growBy(double ratio)
{
    const Point delta(
            (m_max.x - m_mid.x) * ratio,
            (m_max.y - m_mid.y) * ratio,
            (m_max.z - m_mid.z) * ratio);

    m_min -= delta;
    m_max += delta;
}

std::vector<BBox> BBox::explode() const
{
    return std::vector<BBox> {
        getNwd(true), getNed(true), getSwd(true), getSed(true)
    };
}

std::vector<BBox> BBox::explode(std::size_t delta) const
{
    std::vector<BBox> result { *this };

    for (std::size_t i(0); i < delta; ++i)
    {
        result = std::accumulate(
                result.begin(),
                result.end(),
                std::vector<BBox>(),
                [](const std::vector<BBox>& in, const BBox& b)
                {
                    auto out(in);
                    auto next(b.explode());
                    out.insert(out.end(), next.begin(), next.end());
                    return out;
                });
    }

    return result;
}

std::ostream& operator<<(std::ostream& os, const BBox& bbox)
{
    auto flags(os.flags());
    auto precision(os.precision());

    os << std::setprecision(2) << std::fixed;

    os << "[" << bbox.min() << ", " << bbox.max() << "]";

    os << std::setprecision(precision);
    os.flags(flags);

    return os;
}

} // namespace greyhound
} // namepace pdal
