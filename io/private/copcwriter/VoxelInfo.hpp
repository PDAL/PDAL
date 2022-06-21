/******************************************************************************
* Copyright (c) 2021, Hobu Inc. (info@hobu.co)
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

#include <array>
#include <unordered_set>

#include "GridKey.hpp"
#include "VoxelKey.hpp"
#include "OctantInfo.hpp"

namespace pdal
{
namespace copcwriter
{

class VoxelInfo
{
public:
    //ABELL - This probably needs a data structure geared for sparse data.
    //ABELL - Probably the grid should simply be a bitset, but would have to test for
    // tradeoffs with memory/cache since there are a bunch of threads going at once and so on and
    // wouldn't want to burn too much memory. Could make another, more custom structure, but
    // perhaps it's just best to wait until someone cares.
    using Grid = std::unordered_set<GridKey>;

    VoxelInfo(const pdal::BOX3D& fullBounds, const VoxelKey& key) :
        m_fullBounds(fullBounds), m_octant(key)
    {
        //ABELL - This shouldn't be necessary. The key should be in the children
        //  when they're pulled out of the queue.
        for (int i = 0; i < 8; ++i)
            m_children[i].setKey(key.child(i));

        int cells = (int)std::pow(2, key.level());
        m_xWidth = (fullBounds.maxx - fullBounds.minx) / cells;
        m_yWidth = (fullBounds.maxy - fullBounds.miny) / cells;
        m_zWidth = (fullBounds.maxz - fullBounds.minz) / cells;
        // Calculate the bounds of this voxel.
        m_bounds.minx = fullBounds.minx + (key.x() * m_xWidth);
        m_bounds.maxx = m_bounds.minx + m_xWidth;
        m_bounds.miny = fullBounds.miny + (key.y() * m_yWidth);
        m_bounds.maxy = m_bounds.miny + m_yWidth;
        m_bounds.minz = fullBounds.minz + (key.z() * m_zWidth);
        m_bounds.maxz = m_bounds.minz + m_zWidth;

        // Determine spacing between points.

        // We make the child spacing smaller than what we expect as the final spacing since we're
        // going to select points from the grid for the parent.
        if (key == VoxelKey(0, 0, 0, 0))
            m_gridCellWidth = maxWidth() / RootCellCount;
        else
            m_gridCellWidth = maxWidth() / ChildCellCount;

        m_gridXCount = (int)std::ceil((m_bounds.maxx - m_bounds.minx) / m_gridCellWidth);
        m_gridYCount = (int)std::ceil((m_bounds.maxy - m_bounds.miny) / m_gridCellWidth);
        m_gridZCount = (int)std::ceil((m_bounds.maxz - m_bounds.minz) / m_gridCellWidth);
    }

    VoxelKey key() const
        { return m_octant.key(); }

    void initParentOctant()
    {
        if (m_octant.source())
            std::cerr << "Parent octant '" << key() << "' had non-null source before expected.\n";

        int i;
        for (i = 0; i < 8; ++i)
        {
            OctantInfo& o = child(i);
            if (o.source())
            {
                m_octant.source() = o.source()->makeNew();
                break;
            }
        }
        if (i == 8)
            std::cerr << "No non-empty children with which to initialize parent octant '" <<
                key() << "'.\n";
        for (i = 0; i < 8; ++i)
        {
            OctantInfo& o = child(i);
            if (!o.source())
                o.source() = m_octant.source()->makeNew();
        }
    }

    OctantInfo& child(int dir)
        { return m_children[dir]; }

    const OctantInfo& child(int dir) const
        { return m_children[dir]; }

    OctantInfo& octant()
        { return m_octant; }

    bool hasPoints() const
    {
        if (m_octant.numPoints())
            return true;
        for (const OctantInfo& oi : m_children)
            if (oi.numPoints())
                return true;
        return false;
    }

    double minWidth() const
        { return (std::min)((std::min)(m_xWidth, m_yWidth), m_zWidth); }

    double maxWidth() const
        { return (std::max)((std::max)(m_xWidth, m_yWidth), m_zWidth); }

    double xWidth() const
        { return m_xWidth; }

    double yWidth() const
        { return m_yWidth; }

    double zWidth() const
        { return m_zWidth; }

    GridKey gridKey(PointRef p) const
    {
        double x = p.getFieldAs<double>(Dimension::Id::X) - m_bounds.minx;
        double y = p.getFieldAs<double>(Dimension::Id::Y) - m_bounds.miny;
        double z = p.getFieldAs<double>(Dimension::Id::Z) - m_bounds.minz;

        return GridKey((int)(x / m_gridCellWidth), (int)(y / m_gridCellWidth),
            (int)(z / m_gridCellWidth));
    }

    Grid& grid()
        { return m_grid; }

    int gridXCount() const
        { return m_gridXCount; }

    int gridYCount() const
        { return m_gridYCount; }

    int gridZCount() const
        { return m_gridZCount; }

    pdal::BOX3D bounds() const
        { return m_bounds; }

private:
    pdal::BOX3D m_fullBounds;
    pdal::BOX3D m_bounds;
    double m_xWidth;
    double m_yWidth;
    double m_zWidth;
    double m_gridCellWidth;
    int m_gridXCount;
    int m_gridYCount;
    int m_gridZCount;
    std::array<OctantInfo, 8> m_children;
    OctantInfo m_octant;

    Grid m_grid;
};

} // namespace copcwriter
} // namespace pdal
