/******************************************************************************
* Copyright (c) 2016, Connor Manning (connor@hobu.co)
*
* Entwine -- Point cloud indexing
*
* Supporting libraries released under PDAL licensing by Hobu, inc.
*
******************************************************************************/

#include "bounds.hpp"

#include <cmath>
#include <limits>
#include <numeric>
#include <iostream>

namespace pdal
{
namespace entwine
{

Bounds::Bounds(const Point& min, const Point& max)
    : m_min(
            std::min(min.x, max.x),
            std::min(min.y, max.y),
            std::min(min.z, max.z))
    , m_max(
            std::max(min.x, max.x),
            std::max(min.y, max.y),
            std::max(min.z, max.z))
    , m_mid()
{
    setMid();
    if (min.x > max.x || min.y > max.y || min.z > max.z)
    {
        std::cout << "Correcting malformed Bounds" << std::endl;
    }
}

Bounds::Bounds(const Json::Value& json) : m_min(), m_max(), m_mid()
{
    if (!json.isArray() || (json.size() != 4 && json.size() != 6))
    {
        throw std::runtime_error(
            "Invalid JSON Bounds specification: " + json.toStyledString());
    }

    const bool is3d(json.size() == 6);

    if (is3d)
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

    Bounds self(m_min, m_max);
    *this = self;
}

Bounds::Bounds(const Point& center, const double radius)
    : Bounds(center - radius, center + radius)
{ }

Bounds::Bounds(
        const double xMin,
        const double yMin,
        const double zMin,
        const double xMax,
        const double yMax,
        const double zMax)
    : Bounds(Point(xMin, yMin, zMin), Point(xMax, yMax, zMax))
{ }

Bounds::Bounds(
        const double xMin,
        const double yMin,
        const double xMax,
        const double yMax)
    : Bounds(Point(xMin, yMin), Point(xMax, yMax))
{ }

Json::Value Bounds::toJson() const
{
    Json::Value json;

    json.append(m_min.x);
    json.append(m_min.y);
    if (is3d()) json.append(m_min.z);

    json.append(m_max.x);
    json.append(m_max.y);
    if (is3d()) json.append(m_max.z);

    return json;
}

void Bounds::grow(const Bounds& other)
{
    grow(other.min());
    grow(other.max());
}

void Bounds::grow(const Point& p)
{
    m_min.x = std::min(m_min.x, p.x);
    m_min.y = std::min(m_min.y, p.y);
    m_min.z = std::min(m_min.z, p.z);
    m_max.x = std::max(m_max.x, p.x);
    m_max.y = std::max(m_max.y, p.y);
    m_max.z = std::max(m_max.z, p.z);
    setMid();
}

void Bounds::shrink(const Bounds& other)
{
    m_min = Point::max(m_min, other.min());
    m_max = Point::min(m_max, other.max());
    setMid();
}

Bounds Bounds::growBy(double ratio) const
{
    const Point delta(
            (m_max.x - m_mid.x) * ratio,
            (m_max.y - m_mid.y) * ratio,
            (m_max.z - m_mid.z) * ratio);

    return Bounds(m_min - delta, m_max + delta);
}

std::ostream& operator<<(std::ostream& os, const Bounds& bounds)
{
    auto flags(os.flags());
    auto precision(os.precision());

    os << std::setprecision(2) << std::fixed;

    os << "[" << bounds.min() << ", " << bounds.max() << "]";

    os << std::setprecision(precision);
    os.flags(flags);

    return os;
}

} // namespace entwine
} // namespace pdal

