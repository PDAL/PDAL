/******************************************************************************
* Copyright (c) 2011, Michael P. Gerlek (mpg@flaxen.com)
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

#include <assert.h>
#include <iostream>
#include <limits>
#include <vector>

#include <pdal/util/Bounds.hpp>
#include <pdal/util/Utils.hpp>

namespace pdal
{

namespace
{

const double LOWEST = (std::numeric_limits<double>::lowest)();
const double HIGHEST = (std::numeric_limits<double>::max)();

}

void BOX2D::clear()
{
    minx = HIGHEST; miny = HIGHEST;
    maxx = LOWEST; maxy = LOWEST;
}

void BOX3D::clear()
{
    BOX2D::clear();
    minz = HIGHEST;
    maxz = LOWEST;
}

bool BOX2D::empty() const
{
    return  minx == HIGHEST && maxx == LOWEST &&
        miny == HIGHEST && maxy == LOWEST;
}

bool BOX2D::valid() const
{
    return  !empty();
}

bool BOX3D::empty() const
{
    return  BOX2D::empty() && minz == HIGHEST && maxz == LOWEST;
}

bool BOX3D::valid() const
{
    return !empty();
}


BOX2D& BOX2D::grow(double dist)
{
    assert(valid());
    minx -= dist;
    maxx += dist;
    miny -= dist;
    maxy += dist;
    return *this;
}


BOX2D& BOX2D::grow(double x, double y)
{
    if (x < minx) minx = x;
    if (x > maxx) maxx = x;

    if (y < miny) miny = y;
    if (y > maxy) maxy = y;
    return *this;
}

BOX3D& BOX3D::grow(double x, double y, double z)
{
    BOX2D::grow(x, y);
    if (z < minz) minz = z;
    if (z > maxz) maxz = z;
    return *this;
}

const BOX2D& BOX2D::getDefaultSpatialExtent()
{
    static BOX2D v(LOWEST, LOWEST, HIGHEST, HIGHEST);
    return v;
}


const BOX3D& BOX3D::getDefaultSpatialExtent()
{
    static BOX3D v(LOWEST, LOWEST, LOWEST, HIGHEST, HIGHEST, HIGHEST);
    return v;
}

Bounds::Bounds(const BOX3D& box) : m_box(box)
{}


Bounds::Bounds(const BOX2D& box) : m_box(box)
{
    m_box.minz = HIGHEST;
    m_box.maxz = LOWEST;
}

void Bounds::reset(const BOX3D& box)
{
    m_box = box;
}


void Bounds::reset(const BOX2D& box)
{
    m_box.minx = box.minx;
    m_box.maxx = box.maxx;
    m_box.miny = box.miny;
    m_box.maxy = box.maxy;
    m_box.minz = HIGHEST;
    m_box.maxz = LOWEST;
}


// We don't allow implicit conversion from a BOX2D to BOX3D.  Use the explicit
// BOX3D ctor that takes a BOX2D if that's what you want.
BOX3D Bounds::to3d() const
{
    if (!is3d())
        return BOX3D();
    return m_box;
}

BOX2D Bounds::to2d() const
{
    return m_box.to2d();
}

bool Bounds::is3d() const
{
    return (m_box.minz != HIGHEST || m_box.maxz != LOWEST);
}


void Bounds::grow(double x, double y)
{
    if (!is3d())
    {
        m_box.minx = (std::min)(x, m_box.minx);
        m_box.miny = (std::min)(y, m_box.miny);
        m_box.maxx = (std::max)(x, m_box.maxx);
        m_box.maxy = (std::max)(y, m_box.maxy);
    }
}


void Bounds::grow(double x, double y, double z)
{
    m_box.grow(x, y, z);
}


void Bounds::set(const BOX3D& box)
{
    m_box = box;
}


void Bounds::set(const BOX2D& box)
{
    m_box = BOX3D(box);
    m_box.minz = HIGHEST;
    m_box.maxz = LOWEST;
}

namespace
{

template <typename T>
void parsePair(const std::string& s, std::string::size_type& pos,
    double& low, double& high)
{
    low = high = 0;
    const char *start;
    char *end;

    pos += Utils::extractSpaces(s, pos);
    if (s[pos++] != '[')
        throw typename T::error("No opening '[' in range.");

    pos += Utils::extractSpaces(s, pos);
    start = s.data() + pos;
    low = std::strtod(start, &end);
    if (start == end)
        throw typename T::error("No valid minimum value for range.");
    pos += (end - start);

    pos += Utils::extractSpaces(s, pos);
    if (s[pos++] != ',')
        throw typename T::error("No ',' separating minimum/maximum values.");

    pos += Utils::extractSpaces(s, pos);
    start = s.data() + pos;
    high = std::strtod(start, &end);
    if (start == end)
        throw typename T::error("No valid maximum value for range.");
    pos += (end - start);

    pos += Utils::extractSpaces(s, pos);
    if (s[pos++] != ']')
        throw typename T::error("No closing ']' in range.");
}

} // unnamed namespace


// This parses the guts of a 2D range.
void BOX2D::parse(const std::string& s, std::string::size_type& pos)
{
    pos += Utils::extractSpaces(s, pos);
    if (s[pos++] != '(')
        throw error("No opening '('.");
    parsePair<BOX2D>(s, pos, minx, maxx);

    pos += Utils::extractSpaces(s, pos);
    if (s[pos++] != ',')
        throw error("No comma separating 'X' and 'Y' dimensions.");
    parsePair<BOX2D>(s, pos, miny, maxy);

    pos += Utils::extractSpaces(s, pos);
    if (s[pos++] != ')')
        throw error("No closing ')'.");

    pos += Utils::extractSpaces(s, pos);
}


void BOX3D::parse(const std::string& s, std::string::size_type& pos)
{
    pos += Utils::extractSpaces(s, pos);
    if (s[pos++] != '(')
        throw error("No opening '('.");
    parsePair<BOX3D>(s, pos, minx, maxx);

    pos += Utils::extractSpaces(s, pos);
    if (s[pos++] != ',')
        throw error("No comma separating 'X' and 'Y' dimensions.");
    parsePair<BOX3D>(s, pos, miny, maxy);

    pos += Utils::extractSpaces(s, pos);
    if (s[pos++] != ',')
        throw error("No comma separating 'Y' and 'Z' dimensions.");
    parsePair<BOX3D>(s, pos, minz, maxz);

    pos += Utils::extractSpaces(s, pos);
    if (s[pos++] != ')')
        throw error("No closing ')'.");

    pos += Utils::extractSpaces(s, pos);
}


std::istream& operator>>(std::istream& in, BOX2D& box)
{
    std::string s;

    std::getline(in, s);
    std::string::size_type pos(0);

    box.parse(s, pos);
    if (pos != s.size())
        throw BOX2D::error("Invalid characters following valid 2d-bounds.");
    return in;
}


std::istream& operator>>(std::istream& in, BOX3D& box)
{
    std::string s;

    std::getline(in, s);
    std::string::size_type pos(0);

    box.parse(s, pos);
    if (pos != s.size())
        throw BOX3D::error("Invalid characters following valid 3d-bounds.");
    return in;
}

void Bounds::parse(const std::string& s, std::string::size_type& pos)
{
    try
    {
        BOX3D box3d;
        box3d.parse(s, pos);
        set(box3d);
    }
    catch (const BOX3D::error&)
    {
        try
        {
            pos = 0;
            BOX2D box2d;
            box2d.parse(s, pos);
            set(box2d);
        }
        catch (const BOX2D::error& err)
        {
            throw Bounds::error(err.what());
        }
    }
}

std::istream& operator>>(std::istream& in, Bounds& bounds)
{
    std::string s;

    std::getline(in, s);
    std::string::size_type pos(0);

    bounds.parse(s, pos);
    return in;
}

std::ostream& operator<<(std::ostream& out, const Bounds& bounds)
{
    if (bounds.is3d())
        out << bounds.to3d();
    else
        out << bounds.to2d();
    return out;
}

} // namespace pdal
