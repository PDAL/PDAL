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

#include "RialtoCommon.hpp"

#include <pdal/Log.hpp>
#include <pdal/pdal_types.hpp>
#include <pdal/PointView.hpp>
#include <pdal/util/FileUtils.hpp>

#include <cmath>

namespace pdal
{

Tile::Tile(
        int32_t level,
        int32_t tx,
        int32_t ty,
        Rectangle r,
        int32_t maxLevel,
        const PointTableRef table,
        LogPtr log) :
    m_level(level),
    m_tileX(tx),
    m_tileY(ty),
    m_rect(r),
    m_maxLevel(maxLevel),
    m_skip(0),
    m_table(table),
    m_log(log)
{
    assert(m_level >= 0);
    assert(m_tileX >= 0);
    assert(m_tileY >= 0);
    assert(m_maxLevel >= 0);
    m_log->get(LogLevel::Debug1) << "created tb (l=" << m_level
        << ", tx=" << m_tileX
        << ", ty=" << m_tileY
        << ") (slip" << m_skip
        << ")  --  w" << m_rect.m_west
        << " s" << m_rect.m_south
        << " e" << m_rect.m_east
        << " n" << m_rect.m_north << "\n";

    m_children = NULL;

    // level N+1 has 1/4 the points of level N
    //
    // level 3: skip 1
    // level 2: skip 4
    // level 1: skip 16
    // level 0: skip 256

    // max=3, max-level=u
    // 3-3=0  skip 1   4^0
    // 3-2=1  skip 4    4^1
    // 3-1=2  skip 16    4^2
    // 3-0=3  skip 64    4^3
    //
    m_skip = std::pow(4, (m_maxLevel - m_level));
    m_log->get(LogLevel::Debug1) << "level=" << m_level
        << "  skip=" << m_skip << "\n";
}


Tile::~Tile()
{
    if (m_children == NULL)
    {
        for (size_t i=0; i<m_points.size(); ++i)
        {
            char* p = m_points[i];
            delete[] p;
        }
    }
    else
    {
        for (int i=0; i<4; ++i)
        {
            if (m_children[i])
            {
                delete m_children[i];
            }
        }
        delete[] m_children;
    }
}

void Tile::add(PointId pointNumber, char* p, double lon, double lat)
{
    assert(m_rect.contains(lon, lat));

    m_log->get(LogLevel::Debug5) << "-- -- " << pointNumber
        << " " << m_skip
        << " " << (pointNumber % m_skip == 0) << "\n";

    if (pointNumber % m_skip == 0)
    {
        m_points.push_back(p);
    }

    if (m_level == m_maxLevel) return;

    if (!m_children)
    {
        m_children = new Tile*[4];
        m_children[0] = NULL;
        m_children[1] = NULL;
        m_children[2] = NULL;
        m_children[3] = NULL;
    }

    Quad q = m_rect.getQuadrantOf(lon, lat);
    m_log->get(LogLevel::Debug5) << "which=" << q << "\n";

    Tile* child = m_children[q];
    if (child == NULL)
    {
        Rectangle r = m_rect.getQuadrantRect(q);
        switch (q)
        {
            case QuadSW:
                child = new Tile(m_level+1, m_tileX*2, m_tileY*2+1, r, m_maxLevel, m_table, m_log);
                break;
            case QuadNW:
                child = new Tile(m_level+1, m_tileX*2, m_tileY*2, r, m_maxLevel, m_table, m_log);
                break;
            case QuadSE:
                child = new Tile(m_level+1, m_tileX*2+1, m_tileY*2+1, r, m_maxLevel, m_table, m_log);
                break;
            case QuadNE:
                child = new Tile(m_level+1, m_tileX*2+1, m_tileY*2, r, m_maxLevel, m_table, m_log);
                break;
            default:
                assert(0);
        }
        m_children[q] = child;
    }

    child->add(pointNumber, p, lon, lat);
}

void Tile::collectStats(std::vector<int32_t> numTilesPerLevel, std::vector<int64_t> numPointsPerLevel) const
{
    numPointsPerLevel[m_level] += m_points.size();
    ++numTilesPerLevel[m_level];

    for (int i=0; i<4; ++i)
    {
        if (m_children && m_children[i])
        {
            m_children[i]->collectStats(numTilesPerLevel, numPointsPerLevel);
        }
    }
}

void Tile::write(const char* prefix) const
{
    char* filename = new char[strlen(prefix) + 1024];

    sprintf(filename, "%s", prefix);
    FileUtils::createDirectory(filename);

    sprintf(filename, "%s/%d", prefix, m_level);
    FileUtils::createDirectory(filename);

    sprintf(filename, "%s/%d/%d", prefix, m_level, m_tileX);
    FileUtils::createDirectory(filename);

    sprintf(filename, "%s/%d/%d/%d.ria", prefix, m_level, m_tileX, m_tileY);

    m_log->get(LogLevel::Debug1) << "--> " << filename << "\n";

    FILE* fp = fopen(filename, "wb");

    writeData(fp);

    // child mask
    uint8_t mask = 0x0;
    if (m_children)
    {
        if (m_children[QuadSW]) mask += 1;
        if (m_children[QuadSE]) mask += 2;
        if (m_children[QuadNE]) mask += 4;
        if (m_children[QuadNW]) mask += 8;
    }
    fwrite(&mask, 1, 1, fp);

    fclose(fp);

    if (m_children)
    {
        for (int i=0; i<4; ++i)
        {
            if (m_children[i])
            {
                m_children[i]->write(prefix);
            }
        }
    }

    delete[] filename;
}


void Tile::writeData(FILE* fp) const
{
    const PointLayoutPtr layout(m_table.layout());
    for (size_t i=0; i<m_points.size(); ++i)
    {
        char* p = m_points[i];

        for (const auto& dim : layout->dims())
        {
            size_t size = Dimension::size(layout->dimType(dim));

            fwrite(p, size, 1, fp);

            p += size;
        }
    }
}

} // namespace pdal

