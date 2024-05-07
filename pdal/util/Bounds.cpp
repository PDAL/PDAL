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
#include <locale>
#include <sstream>
#include <vector>

#include <pdal/util/Bounds.hpp>
#include <pdal/util/Utils.hpp>

#include <nlohmann/json.hpp>

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
    wkt = "";
}

void BOX3D::clear()
{
    BOX2D::clear();
    minz = HIGHEST;
    maxz = LOWEST;
    wkt = "";
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
    m_box.wkt = box.wkt;
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


bool Bounds::is2d() const
{
    return (valid() && !is3d());
}


bool Bounds::is3d() const
{
    return (m_box.minz != HIGHEST || m_box.maxz != LOWEST);
}


bool Bounds::valid() const
{
    return m_box.valid();
}


bool Bounds::empty() const
{
    return m_box.empty();
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
    if (!is2d())
    {
        m_box.grow(x, y, z);
    }
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

void moveForward(std::istringstream& ss, std::string::size_type count)
{
    for (std::string::size_type i = 0; i < count; i++)
    {
        ss.get();
    }
}

bool discardSpacesBefore(std::istringstream& ss, char nextChar)
{
    Utils::eatwhitespace(ss);
    return Utils::eatcharacter(ss, nextChar);
}

template <typename T>
void parsePair(std::istringstream& ss, double& low, double& high)
{
    low = high = 0;

    if (!discardSpacesBefore(ss, '['))
        throw typename T::error("No opening '[' in range.");

    ss >> low;
    if (!ss.good())
        throw typename T::error("No valid minimum value for range.");

    if (!discardSpacesBefore(ss, ','))
        throw typename T::error("No ',' separating minimum/maximum values.");

    ss >> high;
    if (!ss.good())
        throw typename T::error("No valid maximum value for range.");

    if (!discardSpacesBefore(ss, ']'))
        throw typename T::error("No closing ']' in range.");
}

} // unnamed namespace


// This parses the guts of a 2D range.
void BOX2D::parse(const std::string& s, std::string::size_type& pos)
{

    bool isJson(false);
    bool isArray(false);
    bool isObject(false);
    std::string jsonParseMessage("");
    NL::json b;
    try
    {
        b = NL::json::parse(s);
        isJson = true;
        isArray = b.is_array();
        isObject = b.is_object();
    } catch (std::exception& e)
    {
        jsonParseMessage = e.what();
        isJson = false;
    }

    if (isArray && isJson)
    {
        if (b.size() != 4)
        {
            std::stringstream msg;
            msg << "GeoJSON array size must be 4 for BOX2d. It was " << b.size();
            throw error(msg.str());
        }
        minx = b[0].get<double>();
        miny = b[1].get<double>();
        maxx = b[2].get<double>();
        maxy = b[3].get<double>();

        // parsed. we are done
        pos = s.size();
        return;
    }

    if (isObject && isJson)
    {
        if (!b.contains("minx"))
            throw error("Object must contain 'minx'");
        minx = b["minx"].get<double>();

        if (!b.contains("miny"))
            throw error("Object must contain 'miny'");
        miny = b["miny"].get<double>();

        if (!b.contains("maxx"))
            throw error("Object must contain 'maxx'");
        maxx = b["maxx"].get<double>();

        if (!b.contains("maxy"))
            throw error("Object must contain 'maxy'");
        maxy = b["maxy"].get<double>();

        if (b.contains("crs"))
            wkt = b["crs"].get<std::string>();

        if (b.contains("srs"))
            wkt = b["srs"].get<std::string>();

        // parsed. we are done
        pos = s.size();
        return;

    }

    static thread_local Utils::IStringStreamClassicLocale ss;
    ss.clear();
    ss.str(s);

    moveForward(ss, pos);

    if (!discardSpacesBefore(ss, '('))
        throw error("No opening '('.");

    parsePair<BOX2D>(ss, minx, maxx);

    if (!discardSpacesBefore(ss, ','))
        throw error("No comma separating 'X' and 'Y' dimensions.");

    parsePair<BOX2D>(ss, miny, maxy);

    if (!discardSpacesBefore(ss, ')'))
        throw error("No closing ')'.");

    Utils::eatwhitespace(ss);
    pos = ss.eof() ? s.size() : (std::string::size_type)ss.tellg();
}


void BOX3D::parse(const std::string& s, std::string::size_type& pos)
{

    bool isJson(false);
    bool isArray(false);
    bool isObject(false);
    std::string jsonParseMessage("");
    NL::json b;
    try
    {
        b = NL::json::parse(s);
        isJson = true;
        isArray = b.is_array();
        isObject = b.is_object();
    } catch (std::exception& e)
    {
        jsonParseMessage = e.what();
        isJson = false;
    }

    if (isArray && isJson)
    {
        if (b.size() != 6)
        {
            std::stringstream msg;
            msg << "GeoJSON array must be 6. It was " << b.size();
            throw error(msg.str());
        }
        // Unpack GeoJSON array
        minx = b[0].get<double>();
        miny = b[1].get<double>();
        minz = b[2].get<double>();
        maxx = b[3].get<double>();
        maxy = b[4].get<double>();
        maxz = b[5].get<double>();

        // Parsed. We are done.
        pos = s.size();
        return;
    }

    if (isObject && isJson)
    {
        if (!b.contains("minx"))
            throw error("Object must contain 'minx'");
        minx = b["minx"].get<double>();

        if (!b.contains("miny"))
            throw error("Object must contain 'miny'");
        miny = b["miny"].get<double>();

        if (!b.contains("maxx"))
            throw error("Object must contain 'maxx'");
        maxx = b["maxx"].get<double>();

        if (!b.contains("maxy"))
            throw error("Object must contain 'maxy'");
        maxy = b["maxy"].get<double>();

        if (b.contains("minz"))
        {
            if (!b.contains("maxx"))
                throw error("Object must contain 'maxz' if 'minz' is provided");
            minz = b["minz"].get<double>();
            maxz = b["maxz"].get<double>();
        }

        if (b.contains("crs"))
            wkt = b["crs"].get<std::string>();

        if (b.contains("srs"))
            wkt = b["srs"].get<std::string>();

        // Parsed. We are done.
        pos = s.size();
        return;
    }

    static thread_local Utils::IStringStreamClassicLocale ss;
    ss.clear();
    ss.str(s);


    moveForward(ss, pos);

    if (!discardSpacesBefore(ss, '('))
        throw error("No opening '('.");

    parsePair<BOX3D>(ss, minx, maxx);

    if (!discardSpacesBefore(ss, ','))
        throw error("No comma separating 'X' and 'Y' dimensions.");

    parsePair<BOX3D>(ss, miny, maxy);

    if (!discardSpacesBefore(ss, ','))
        throw error("No comma separating 'Y' and 'Z' dimensions.");

    parsePair<BOX3D>(ss, minz, maxz);

    if (!discardSpacesBefore(ss, ')'))
        throw error("No closing ')'.");

    Utils::eatwhitespace(ss);
    pos = ss.eof() ? s.size() : (std::string::size_type)ss.tellg();
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

    try
    {
        BOX3D box3d;
        box.parse(s, pos);
    }
    catch (const BOX3D::error&)
    {
        try
        {
            pos = 0;
            BOX2D box2d;
            box2d.parse(s, pos);
            box = BOX3D(box2d);
        }
        catch (const BOX2D::error& err)
        {
            throw BOX3D::error(err.what());
        }
    }

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
