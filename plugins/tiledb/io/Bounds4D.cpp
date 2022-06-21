/******************************************************************************
* Copyright (c) 2021 TileDB, Inc.
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

#include "Bounds4D.hpp"
#include <pdal/PluginHelper.hpp>

namespace pdal
{

namespace
{

const double LOWEST = (std::numeric_limits<double>::lowest)();
const double HIGHEST = (std::numeric_limits<double>::max)();

}

bool BOX4D::empty() const
{
    return  BOX3D::empty() && mintm == HIGHEST && maxtm == LOWEST;
}

bool BOX4D::valid() const
{
    return !empty();
}

BOX4D& BOX4D::grow(double x, double y, double z, double tm)
{
    BOX3D::grow(x, y, z);
    if (tm < mintm) mintm = tm;
    if (tm > maxtm) maxtm = tm;
    return *this;
}

void BOX4D::clear()
{
    BOX3D::clear();
    mintm = HIGHEST;
    maxtm = LOWEST;
}

const BOX4D& BOX4D::getDefaultSpatialExtent()
{
    static BOX4D v(LOWEST, LOWEST, LOWEST, LOWEST,
                   HIGHEST, HIGHEST, HIGHEST, HIGHEST);
    return v;
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

}

void BOX4D::parse(const std::string& s, std::string::size_type& pos)
{
    pos += Utils::extractSpaces(s, pos);
    if (s[pos++] != '(')
        throw error("No opening '('.");
    parsePair<BOX4D>(s, pos, minx, maxx);

    pos += Utils::extractSpaces(s, pos);
    if (s[pos++] != ',')
        throw error("No comma separating 'X' and 'Y' dimensions.");
    parsePair<BOX4D>(s, pos, miny, maxy);

    pos += Utils::extractSpaces(s, pos);
    if (s[pos++] != ',')
        throw error("No comma separating 'Y' and 'Z' dimensions.");
    parsePair<BOX4D>(s, pos, minz, maxz);

    pos += Utils::extractSpaces(s, pos);
    if (s[pos] != ',' && s[pos] != ')')
        throw error("No comma separating 'Z' and 'time' dimensions.");
    else if (s[pos++] != ')')
    {
        parsePair<BOX4D>(s, pos, mintm, maxtm);
        pos++;
    }

    pos += Utils::extractSpaces(s, pos);
}

std::istream& operator>>(std::istream& in, BOX4D& box)
{
    std::string s;

    std::getline(in, s);
    std::string::size_type pos(0);

    box.parse(s, pos);
    if (pos != s.size())
        throw BOX4D::error("Invalid characters following valid 4d-bounds.");
    return in;
}

Bounds4D::Bounds4D(const BOX4D& box) : m_box(box)
{}

Bounds4D::Bounds4D(const BOX3D& box)
{
    m_box.minx = box.minx;
    m_box.maxx = box.maxx;
    m_box.miny = box.miny;
    m_box.maxy = box.maxy;
    m_box.minz = box.minz;
    m_box.maxz = box.maxz;
    m_box.mintm = HIGHEST;
    m_box.maxtm = LOWEST;
}

Bounds4D::Bounds4D(const BOX2D& box)
{
    m_box.minx = box.minx;
    m_box.maxx = box.maxx;
    m_box.miny = box.miny;
    m_box.maxy = box.maxy;
    m_box.minz = HIGHEST;
    m_box.maxz = LOWEST;
    m_box.mintm = HIGHEST;
    m_box.maxtm = LOWEST;
}

BOX4D Bounds4D::to4d() const
{
    if (!is4d())
        return BOX4D();
    return m_box;
}

BOX3D Bounds4D::to3d() const
{
    if (is2d())
        return BOX3D();
    return m_box.to3d();
}

BOX2D Bounds4D::to2d() const
{
    return m_box.to2d();
}

bool Bounds4D::is4d() const
{
    return (m_box.mintm != HIGHEST || m_box.maxtm != LOWEST);
}

bool Bounds4D::is3d() const
{
    return ((m_box.minz != HIGHEST || m_box.maxz != LOWEST) && !is4d());
}

bool Bounds4D::is2d() const
{
    return (valid() && !is4d() && !is3d());
}

bool Bounds4D::empty() const
{
    return m_box.empty();
}

bool Bounds4D::valid() const
{
    return m_box.valid();
}

void Bounds4D::reset(const BOX4D& box)
{
    m_box = box;
}

void Bounds4D::reset(const BOX3D& box)
{
    m_box.minx = box.minx;
    m_box.maxx = box.maxx;
    m_box.miny = box.miny;
    m_box.maxy = box.maxy;
    m_box.minz = box.minz;
    m_box.maxz = box.maxz;
    m_box.mintm = HIGHEST;
    m_box.maxtm = LOWEST;
}

void Bounds4D::reset(const BOX2D& box)
{
    m_box.minx = box.minx;
    m_box.maxx = box.maxx;
    m_box.miny = box.miny;
    m_box.maxy = box.maxy;
    m_box.minz = HIGHEST;
    m_box.maxz = LOWEST;
    m_box.mintm = HIGHEST;
    m_box.maxtm = LOWEST;
}

void Bounds4D::grow(double x, double y)
{
    if (!is4d() && !is3d())
    {
        m_box.minx = (std::min)(x, m_box.minx);
        m_box.miny = (std::min)(y, m_box.miny);
        m_box.maxx = (std::max)(x, m_box.maxx);
        m_box.maxy = (std::max)(y, m_box.maxy);
    }
}

void Bounds4D::grow(double x, double y, double z)
{
    if (!is4d() && !is2d())
    {
        m_box.minx = (std::min)(x, m_box.minx);
        m_box.miny = (std::min)(y, m_box.miny);
        m_box.minz = (std::min)(z, m_box.minz);
        m_box.maxx = (std::max)(x, m_box.maxx);
        m_box.maxy = (std::max)(y, m_box.maxy);
        m_box.maxz = (std::max)(z, m_box.maxz);
    }
}

void Bounds4D::grow(double x, double y, double z, double tm)
{
    if (!is3d() && !is2d())
    {
        m_box.grow(x, y, z, tm);
    }
}

void Bounds4D::parse(const std::string& s, std::string::size_type& pos)
{
    try
    {
        BOX4D box4d;
        box4d.parse(s, pos);
        set(box4d);
    }
    catch (const BOX4D::error&)
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
                BOX2D_ box2d;
                box2d.parse(s, pos);
                set(box2d);
            }
            catch (const BOX2D::error& err)
            {
                throw Bounds4D::error(err.what());
            }
        }
    }
}

void Bounds4D::set(const BOX4D& box)
{
    m_box = box;
}

void Bounds4D::set(const BOX3D& box)
{
    m_box = BOX4D(box);
    m_box.mintm = HIGHEST;
    m_box.maxtm = LOWEST;
}

void Bounds4D::set(const BOX2D_& box)
{
    m_box = BOX4D(box);
    m_box.minz = HIGHEST;
    m_box.maxz = LOWEST;
    m_box.mintm = HIGHEST;
    m_box.maxtm = LOWEST;
}

std::istream& operator>>(std::istream& in, Bounds4D& bounds)
{
    std::string s;

    std::getline(in, s);
    std::string::size_type pos(0);

    bounds.parse(s, pos);
    return in;
}

std::ostream& operator<<(std::ostream& out, const Bounds4D& bounds)
{
    if (bounds.is4d())
        out << bounds.to4d();
    else if (bounds.is3d())
        out << bounds.to3d();
    else
        out << bounds.to2d();
    return out;
}

class Metadata4D : public Metadata
{
std::string inferType(const std::string& val);
};

std::string Metadata4D::inferType(const std::string& val)
{
    size_t pos;

    long l = 0;
    try
    {
        pos = 0;
        l = std::stol(val, &pos);
    }
    catch (std::invalid_argument&)
    {}
    if (pos == val.length())
        return (l < 0 ? "nonNegativeInteger" : "integer");

    try
    {
        pos = 0;
        std::stod(val, &pos);
    }
    catch(std::invalid_argument&)
    {}

    if (pos == val.length())
        return "double";

    BOX2D b2d;
    std::istringstream iss1(val);
    try
    {
        iss1 >> b2d;
        if (iss1.good())
            return "bounds";
    }
    catch (const BOX2D::error&)
    {}

    BOX3D b3d;
    std::istringstream iss2(val);
    try
    {
        iss2 >> b3d;
        if (iss2.good())
            return "bounds";
    }
    catch (const BOX3D::error&)
    {}

    BOX4D b4d;
    std::istringstream iss3(val);
    try
    {
        iss3 >> b4d;
        if (iss3.good())
            return "bounds";
    }
    catch (const BOX4D::error&)
    {}

    if (val == "true" || val == "false")
        return "boolean";

    try
    {
        SpatialReference s(val);
        return "spatialreference";
    }
    catch (pdal_error&)
    {
    }

    Uuid uuid(val);
    if (!uuid.isNull())
        return "uuid";

    return "string";
}

}
