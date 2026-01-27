/******************************************************************************
* Copyright (c) 2025, Hobu Inc.
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

#include "PointGrid.hpp"

namespace pdal
{

PointGrid::NeighborResults PointGrid::findKnn(Eigen::Vector2d pos, point_count_t k) const
{
    NeighborResults results;
    results.reserve(k);

    auto [iStart, jStart] = toIJ(pos(0), pos(1));
    const Cell& cStart = cell(iStart, jStart);
    if (k > m_view.size()) // edge case
        k = m_view.size();

    // Setting the max distance to the size of a cell so all points are checked to start.
    double curMaxDist = std::sqrt(m_xlen * m_xlen + m_ylen * m_ylen);
    std::vector<uint32_t> skip;
    std::vector<uint32_t> keys = { key(iStart, jStart) };
    while (keys.size())
    {
        for (auto key : keys)
        {
            skip.push_back(key);
            const Cell& c = m_cells[key];
            for (PointId id : c)
            {
                Eigen::Vector2d pos2(m_view.getFieldAs<double>(Dimension::Id::X, id),
                    m_view.getFieldAs<double>(Dimension::Id::Y, id));
                double dist2 = (pos2 - pos).squaredNorm();
                if (dist2 < curMaxDist)
                {
                    //std::cout << "found neighbor: " << id << std::endl;
                    results.emplace_back(id, dist2);
                }
            }
            std::sort(results.begin(), results.end(), [](const std::pair<PointId, double>& a,
                const std::pair<PointId, double>& b) {
                return a.second < b.second;
            });
            if (results.size() >= k) // resize is dumb if size == k but whatever
            {
                results.resize(k);
                curMaxDist = results.back().second;
            }
            else if (skip.size() > 1) // Probably impossible edge case if we find nothing; Just get everything
                curMaxDist = std::sqrt(std::pow((m_bounds.maxx - m_bounds.minx), 2) + 
                    std::pow((m_bounds.maxy - m_bounds.miny), 2));
        }
        keys = nextCells(pos, curMaxDist, skip);
    }
    return results;
}

// Basically doing the radius method. returning all these cells is bad 
// but returning ij would be annoying.
std::vector<uint32_t> PointGrid::nextCells(Eigen::Vector2d pos,
    double maxDist, std::vector<uint32_t>& skip) const 
{
    BOX2D box;
    box.grow(pos(0), pos(1));
    box.grow(maxDist);
    box.clip(bounds());

    auto [imin, jmin] = toIJ(box.minx, box.miny);
    auto [imax, jmax] = toIJ(box.maxx, box.maxy);

    std::vector<uint32_t> cells;
    //cells.reserve((imax - imin + 1) * (jmax - jmin + 1));
    for (uint16_t i = imin; i <= imax; ++i)
        for (uint16_t j = jmin; j <= jmax; ++j)
            {
                uint32_t curKey = key(i, j);
                if (std::find(skip.begin(), skip.end(), curKey) == skip.end())
                    cells.emplace_back(curKey);
            }
    return cells;
}

PointIdList PointGrid::findNeighbors3d(Eigen::Vector3d pos, double radius) const
{
    BOX2D extent;
    extent.grow(pos(0), pos(1));  // Start with center point
    extent.grow(radius);          // Expand by radius
    extent.clip(bounds());        // Clip to grid bounds.

    // Find IJ bounding box.
    auto [imin, jmin] = toIJ(extent.minx, extent.miny);
    auto [imax, jmax] = toIJ(extent.maxx, extent.maxy);

    PointIdList neighbors;
    double radius2 = radius * radius;  // We use square distance

    for (uint16_t i = imin; i <= imax; ++i)
        for (uint16_t j = jmin; j <= jmax; ++j)
        {
            const Cell& c = cell(i, j);
            for (PointId id : c)
            {
                Eigen::Vector3d pos2(m_view.getFieldAs<double>(Dimension::Id::X, id),
                    m_view.getFieldAs<double>(Dimension::Id::Y, id),
                    m_view.getFieldAs<double>(Dimension::Id::Z, id));

                if ((pos - pos2).squaredNorm() < radius2)
                    neighbors.push_back(id);
            }
        }

    return neighbors;
}

PointIdList PointGrid::findNeighbors(BOX2D extent) const
{
    extent.clip(bounds());

    PointIdList neighbors;

    // Find IJ bounding box.
    auto [imin, jmin] = toIJ(extent.minx, extent.miny);
    auto [imax, jmax] = toIJ(extent.maxx, extent.maxy);

    for (uint16_t i = imin; i <= imax; ++i)
        for (uint16_t j = jmin; j <= jmax; ++j)
        {
            const Cell& c = cell(i, j);

            // If the entire cell is in the extent, append all points.
            if (extent.contains(bounds(i, j)))
                neighbors.insert(neighbors.end(), c.begin(), c.end());
            // Otherwise, check each point to make sure it's in the xtent.
            else
                for (PointId id : c)
                {
                    double x = m_view.getFieldAs<double>(Dimension::Id::X, id);
                    double y = m_view.getFieldAs<double>(Dimension::Id::Y, id);
                    if (extent.contains(x, y))
                        neighbors.push_back(id);
                }
        }
    return neighbors;
}

PointIdList PointGrid::findNeighbors(PointRef& point, double radius) const
{
    BOX2D extent;
    extent.grow(point.getFieldAs<double>(Dimension::Id::X), 
        point.getFieldAs<double>(Dimension::Id::Y));
    extent.grow(radius);

    return findNeighbors(extent);
}

} // namespace pdal
