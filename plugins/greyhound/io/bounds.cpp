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

#include "bounds.hpp"

#include <cmath>
#include <limits>
#include <numeric>
#include <iostream>

#include <nlohmann/json.hpp>

namespace pdal
{
namespace entwine
{

Bounds::Bounds(const Point& minimum, const Point& maximum)
    : m_min(
            (std::min)(minimum.x, maximum.x),
            (std::min)(minimum.y, maximum.y),
            (std::min)(minimum.z, maximum.z))
    , m_max(
            (std::max)(minimum.x, maximum.x),
            (std::max)(minimum.y, maximum.y),
            (std::max)(minimum.z, maximum.z))
    , m_mid()
{
    setMid();
    if (minimum.x > maximum.x || minimum.y > maximum.y || minimum.z > maximum.z)
    {
        std::cout << "Correcting malformed Bounds" << std::endl;
    }
}

Bounds::Bounds(const NL::json& j)
{
    if (!j.is_array() || (j.size() != 4 && j.size() != 6))
    {
        throw std::runtime_error(
            "Invalid JSON Bounds specification: " + j.dump());
    }

    const bool is3d(j.size() == 6);

    if (is3d)
    {
        m_min = Point(j.at(0).get<double>(), j.at(1).get<double>(),
             j.at(2).get<double>());
        m_max = Point(j.at(3).get<double>(), j.at(4).get<double>(),
             j.at(5).get<double>());
    }
    else
    {
        m_min = Point(j.at(0).get<double>(), j.at(1).get<double>());
        m_min = Point(j.at(2).get<double>(), j.at(3).get<double>());
    }
    setMid();
}

Bounds::Bounds(const Point& center, const double radius)
    : Bounds(center - radius, center + radius)
{}

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

NL::json Bounds::toJson() const
{
    if (is3d())
        return { m_min.x, m_min.y, m_min.z, m_max.x, m_max.y, m_max.z };
    return { m_min.x, m_min.y, m_max.x, m_max.y };
}

void Bounds::grow(const Bounds& other)
{
    grow(other.minimum());
    grow(other.maximum());
}

void Bounds::grow(const Point& p)
{
    m_min.x = (std::min)(m_min.x, p.x);
    m_min.y = (std::min)(m_min.y, p.y);
    m_min.z = (std::min)(m_min.z, p.z);
    m_max.x = (std::max)(m_max.x, p.x);
    m_max.y = (std::max)(m_max.y, p.y);
    m_max.z = (std::max)(m_max.z, p.z);
    setMid();
}

void Bounds::shrink(const Bounds& other)
{
    m_min = Point::maximum(m_min, other.minimum());
    m_max = Point::minimum(m_max, other.maximum());
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

    os << "[" << bounds.minimum() << ", " << bounds.maximum() << "]";

    os << std::setprecision(precision);
    os.flags(flags);

    return os;
}

} // namespace entwine
} // namespace pdal

