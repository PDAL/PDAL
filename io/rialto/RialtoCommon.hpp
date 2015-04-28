/******************************************************************************
* Copyright (c) 2014-2015, RadiantBlue Technologies, Inc.
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

#pragma once

#include <pdal/pdal_types.hpp>
#include <pdal/Writer.hpp>

#include <cassert>
#include <cstdint>
#include <vector>

namespace pdal
{

enum Quad
{
    QuadSW=0, QuadNW=1, QuadSE=2, QuadNE=3,
};

class Rectangle
{
public:
    Rectangle() : m_north(0.0), m_south(0.0), m_east(0.0), m_west(0.0), m_midx(0.0), m_midy(0.0)
        {}

    Rectangle(double w, double s, double e, double n) :
        m_north(n), m_south(s), m_east(e), m_west(w), m_midx((w+e)*0.5), m_midy((s+n)*0.5)
        {}

    Rectangle(const Rectangle& r) :
        m_north(r.m_north), m_south(r.m_south), m_east(r.m_east), m_west(r.m_west),
        m_midx((r.m_west+r.m_east)*0.5), m_midy((r.m_south+r.m_north)*0.5)
        {}

    Rectangle& operator=(const Rectangle& r)
    {
        m_north = r.m_north;
        m_south = r.m_south;
        m_east = r.m_east;
        m_west = r.m_west;
        m_midx = r.m_midx;
        m_midy = r.m_midy;
        return *this;
    }

    Quad getQuadrantOf(double lon, double lat)
    {
        // NW=1  NE=3
        // SW=0  SE=2
        bool lowX = (lon <= m_midx);
        bool lowY = (lat <= m_midy);

        if (lowX && lowY)
            return QuadSW;
        else if (lowX && !lowY)
            return QuadNW;
        else if (!lowX && lowY)
            return QuadSE;
        else
            return QuadNE;
    }

    Rectangle getQuadrantRect(Quad q)
    {
        assert(q == QuadSW || q == QuadNW || q == QuadSE || q == QuadNE);

        // w s e n
        if (q == QuadSW)
            return Rectangle(m_west, m_south, m_midx, m_midy);
        else if (q == QuadNW)
            return Rectangle(m_west, m_midy, m_midx, m_north);
        else if (q == QuadSE)
            return Rectangle(m_midx, m_south, m_east, m_midy);
        else
            return Rectangle(m_midx, m_midy, m_east, m_north);
    }

    bool contains(double lon, double lat) const
    {
        return (lon >= m_west) && (lon <= m_east) &&
               (lat >= m_south) && (lat <= m_north);
    }

    double m_north, m_south, m_east, m_west, m_midx, m_midy;
};

class Tile
{
public:
    Tile(int32_t level, int32_t tx, int32_t ty, Rectangle r, int32_t maxLevel,
        const PointTableRef table, LogPtr log);
    ~Tile();

    std::vector<char*>& points()
        { return m_points; }

    size_t numPoints() const
        { return m_points.size(); }

    void add(PointId pointNumber, char* data, double lon, double lat);
    void collectStats(std::vector<int32_t> numTilesPerLevel, std::vector<int64_t> numPointsPerLevel) const;
    void write(const char* dir) const;
    void writeData(FILE*) const;

private:
    int32_t m_level;
    int32_t m_tileX;
    int32_t m_tileY;
    std::vector<char*> m_points;
    Tile** m_children;
    Rectangle m_rect;
    int32_t m_maxLevel;
    int64_t m_skip;
    const PointTableRef m_table;
    LogPtr m_log;
};

} // namespace pdal

