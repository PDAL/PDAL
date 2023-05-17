/******************************************************************************
* Copyright (c) 2019, Hobu Inc. (info@hobu.co)
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
*     * Neither the name of Hobu, Inc. names of its contributors may be
*       used to endorse or promote products derived from this software
*       without specific prior written permission.
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

#include <limits>

#include <pdal/SrsBounds.hpp>

namespace
{

const double LOWEST = (std::numeric_limits<double>::lowest)();
const double HIGHEST = (std::numeric_limits<double>::max)();

}

namespace pdal
{

SrsBounds::SrsBounds(const BOX3D& box) : Bounds(box)
{
    const std::string& wkt = box.to2d().wkt;
    if (wkt.size())
       m_srs.set(wkt);
}


SrsBounds::SrsBounds(const BOX3D& box, const SpatialReference& srs) :
    Bounds(box), m_srs(srs)
{
    const std::string& wkt = box.to2d().wkt;
    if (wkt.size())
       m_srs.set(wkt);
}


SrsBounds::SrsBounds(const BOX2D& box) : Bounds(box)
{
    const std::string& wkt = box.wkt;
    if (wkt.size())
       m_srs.set(wkt);
}


SrsBounds::SrsBounds(const BOX2D& box, const SpatialReference& srs) :
    Bounds(box), m_srs(srs)
{}


void SrsBounds::parse(const std::string& s, std::string::size_type& pos)
{
    Bounds::parse(s, pos);
    pos += Utils::extractSpaces(s, pos);
    m_srs.set(to2d().wkt);
    if (pos == s.size())
        return;

    if (s[pos++] != '/')
        throw Bounds::error("Invalid character following valid bounds box.");

    pos += Utils::extractSpaces(s, pos);
    SpatialReference srs;
    m_srs.parse(s, pos);
    pos += Utils::extractSpaces(s, pos);
}

std::ostream& operator << (std::ostream& out, const SrsBounds& srsBounds)
{
    out << static_cast<const Bounds&>(srsBounds);
    out << " / " << srsBounds.m_srs;
    return out;
}

} // namespace pdal
