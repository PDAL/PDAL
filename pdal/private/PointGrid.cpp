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

static int POINT = 0;

namespace pdal
{


/// 2D ///

PointGrid::DistanceResults PointGrid::knnSearch(double x, double y, point_count_t k) const
{
    Eigen::Vector2d pos(x, y);
    DistanceResults results;
    results.reserve(k);

    // Find the starting cell
    auto [iStart, jStart] = toIJBounded(x, y);
    if (k > m_view.size()) // edge case
        k = m_view.size();

    // Starting off as the diagonal of a single cell.
    double curMaxDist = m_xlen * m_xlen + m_ylen * m_ylen;
    // If the point of interest is outside the grid, make all our distances in relation to a dummy
    // point at the center of our first cell.
    if (!bounds().contains(x, y))
    {
        BOX2D cellBounds = bounds(iStart, jStart);
        Eigen::Vector2d cellCenter((cellBounds.maxx - cellBounds.minx) / 2,
            (cellBounds.maxy - cellBounds.miny) / 2);
        double dist = (cellCenter - pos).squaredNorm();
        curMaxDist += dist;
    }
    // Keep track of cells we've already checked
    std::vector<uint32_t> skip;
    // This will contain all cells on the queue as keys for simpler lookups than Cell or ij
    std::vector<uint32_t> keys = { key(iStart, jStart) };
    // Start with the first cell, then move outwards if we need to.
    while (keys.size())
    {
        for (auto key : keys)
        {
            skip.push_back(key);
            const Cell& c = m_cells[key];
            // Check each point is below our current max distance
            for (PointId id : c)
            {
                Eigen::Vector2d pos2(m_view.getFieldAs<double>(Dimension::Id::X, id),
                    m_view.getFieldAs<double>(Dimension::Id::Y, id));
                double dist2 = (pos2 - pos).squaredNorm();
                if (dist2 < curMaxDist)
                    results.emplace_back(id, dist2);
            }
            // Sort by distance
            std::sort(results.begin(), results.end());
            // If we've found enough, we can use a more precise max distance as our radius
            // in nextCells(). If not, use the one that was set at the start (assuming this is the
            // first iteration).
            if (results.size() >= k)
            {
                results.resize(k);
                curMaxDist = results.back().sqr_dist;
            }
            // If it's not the first iteration and still fails to find k, grow the radius
            // by a bit. If we already tried and we reached the max distance, we're done
            else if (skip.size() > 1)
                curMaxDist = curMaxDist * 1.5;
        }
        // Get the next cells. If there are none, we're done
        keys = nextCells(pos, curMaxDist, skip);
    }
    //!! Warn if we didn't find enough?
    return results;
}


PointIdList PointGrid::boxEncloses(BOX2D extent) const
{
    PointIdList neighbors;

    if (!extent.overlaps(bounds()))
        return neighbors;

    extent.clip(bounds());

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
            // Otherwise, check each point to make sure it's in the extent.
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


PointIdList PointGrid::radius(double x, double y, double radius) const
{
    PointIdList neighbors;
    DistanceResults results = radiusSearch(x, y, radius);
    for (size_t i = 0; i < results.size(); ++i)
        neighbors.push_back(results[i].index);
    return neighbors;
}


PointGrid::DistanceResults PointGrid::radiusSearch(double x, double y, double radius) const
{
    Eigen::Vector2d pos(x, y);
    DistanceResults results;
    double radius2 = radius * radius;  // We use square distance

    for (uint32_t key : radiusCells(pos, radius))
        for (PointId id : m_cells[key])
        {
            Eigen::Vector2d pos2(m_view.getFieldAs<double>(Dimension::Id::X, id),
                m_view.getFieldAs<double>(Dimension::Id::Y, id));
            double dist = (pos - pos2).squaredNorm();
            if (dist < radius2)
                results.emplace_back(id, dist);
        }

    std::sort(results.begin(), results.end());

    return results;
}


/// 3D ///


void PointGrid::processCellPoints(Eigen::Vector3d pos,
    const DistanceResults& possibleCells, KnnResults& results) const
{
    for (const DistanceResult& possibleCell : possibleCells)
    {
        uint32_t k = static_cast<uint32_t>(possibleCell.index);
        const Cell& c = m_cells[k];
        for (PointId id : c)
        {
            Eigen::Vector3d pos2(m_view.getFieldAs<double>(Dimension::Id::X, id),
                m_view.getFieldAs<double>(Dimension::Id::Y, id),
                m_view.getFieldAs<double>(Dimension::Id::Z, id));
            results.tryInsert(id, (pos - pos2).squaredNorm());
        }
        // Need to bail here if the results are full and the distance is less than that
        // of the next possibleCell.
    }
}

PointGrid::DistanceResults PointGrid::knnSearch(double x, double y, double z,
    point_count_t k) const
{
    if (k > m_view.size()) // edge case
        k = m_view.size();

POINT++;
    Eigen::Vector3d pos(x, y, z);
    KnnResults results(k);

    // Find the starting cell
    auto [iStart, jStart] = toIJBounded(x, y);
    uint32_t cellKey = key(iStart, jStart);

    // Keep track of cells we've already checked
    std::vector<uint32_t> skip;

    // Stick a start cell on the list of cells.
    DistanceResults possibleCells;
    possibleCells.emplace_back(cellKey, 0);
    skip.push_back(cellKey);

    double maxDist = 0;
    while (possibleCells.size())
    {
        processCellPoints(pos, possibleCells, results);
        if (results.full())
        {
            maxDist = std::sqrt(results.maxDistance());
//            std::cerr << "*** FULL " << maxDist << "!\n";
        }
        else
        {
//            std::cerr << "\t*** NOT FULL";
            if (maxDist == 0 && !bounds().contains(x, y))
            {
//                std::cerr << " - OUTSIDE";
                maxDist = boundsDistance(x, y, bounds(iStart, jStart));
            }
//            std::cerr << "!\n";
            maxDist += (std::min)(m_xlen, m_ylen);
        }
        possibleCells = nextClosestCells({x, y}, maxDist, skip);
    }
    return results.sortedResults();
}


PointIdList PointGrid::neighbors(double x, double y, double z, point_count_t k, int stride) const
{
    // Account for input buffer size smaller than requested number of
    // neighbors, then determine the number of neighbors to extract based
    // on the desired stride.
    k = (std::min)(m_view.size(), k);
    point_count_t k2 = stride * k;

    // Extract k*stride neighbors, then return only k, selecting every nth
    // neighbor at the given stride.
    DistanceResults results = knnSearch(x, y, z, k2);
    PointIdList output(k);

    // We can always multiply by stride, since we are looping over results anyway.
    for (size_t i = 0; i < k; ++i)
        output[i] = results[i * stride].index;

    return output;
}


PointIdList PointGrid::radius(double x, double y, double z, double radius) const
{
    PointIdList neighbors;
    DistanceResults results = radiusSearch(x, y, z, radius);
    for (size_t i = 0; i < results.size(); ++i)
        neighbors.push_back(results[i].index);
    return neighbors;
}


PointGrid::DistanceResults PointGrid::radiusSearch(double x, double y, double z,
    double radius) const
{
    Eigen::Vector3d pos(x, y, z);
    DistanceResults results;
    double radius2 = radius * radius;  // We use square distance

    for (uint32_t key : radiusCells({pos(0), pos(1)}, radius))
        for (PointId id : m_cells[key])
        {
            Eigen::Vector3d pos2(m_view.getFieldAs<double>(Dimension::Id::X, id),
                m_view.getFieldAs<double>(Dimension::Id::Y, id),
                m_view.getFieldAs<double>(Dimension::Id::Z, id));
            double dist = (pos - pos2).squaredNorm();
            if (dist < radius2)
                results.emplace_back(id, dist);
        }

    std::sort(results.begin(), results.end());

    return results;
}


/// Flex ///

PointGrid::KnnResults PointGrid::knnSearch(PointRef& p, point_count_t k) const
{
    Eigen::VectorXd pos(m_dims.size());
    for (size_t i = 0; i < m_dims.size(); ++i)
        pos(i) = p.getFieldAs<double>(m_dims[i]);
     return knnSearch(pos, k);
}

PointGrid::DistanceResults PointGrid::radiusSearch(PointRef& p, double radius) const
{
    Eigen::VectorXd pos(m_dims.size());
    for (size_t i = 0; i < m_dims.size(); ++i)
        pos(i) = p.getFieldAs<double>(m_dims[i]);
    return radiusSearch(pos, radius);
}

PointIdList PointGrid::neighbors(PointRef& p, point_count_t k, int stride) const
{
    // Account for input buffer size smaller than requested number of
    // neighbors, then determine the number of neighbors to extract based
    // on the desired stride.
    k = (std::min)(m_view.size(), k);
    point_count_t k2 = stride * k;

    Eigen::VectorXd pos(m_dims.size());

    for (size_t i = 0; i < m_dims.size(); ++i)
        pos(i) = p.getFieldAs<double>(m_dims[i]);

    // Extract k*stride neighbors, then return only k, selecting every nth
    // neighbor at the given stride.
    PointIdList output(k);
    std::vector<double> out_dist_sqr(k2);
    knnSearch(pos, k2).sortedResults(output, out_dist_sqr);

    if (stride > 1)
    {
        for (size_t i = 1; i < k; ++i)
            output[i] = output[i * stride];
        output.resize(k);
    }

    return output;
}

PointGrid::KnnResults PointGrid::knnSearch(Eigen::VectorXd pos, point_count_t k) const
{
    // could check this elsewhere
    assert(pos.rows() == m_dims.size());
    KnnResults results(k);

    // Find the starting cell
    auto [iStart, jStart] = toIJBounded(pos(0), pos(1));
    uint32_t cellKey = key(iStart, jStart);

    // Keep track of cells we've already checked
    std::vector<uint32_t> skip;

    // Stick a start cell on the list of cells.
    DistanceResults possibleCells;
    possibleCells.emplace_back(cellKey, 0);
    skip.push_back(cellKey);

    double maxDist = 0;
    double x = pos(0);
    double y = pos(1);
    while (possibleCells.size())
    {
        processCellPointsXd(pos, possibleCells, results);
    
        if (results.full())
            maxDist = std::sqrt(results.maxDistance());

        else
        {
            if (maxDist == 0 && !bounds().contains(x, y))
                maxDist = boundsDistance(x, y, bounds(iStart, jStart));
            maxDist += (std::min)(m_xlen, m_ylen);
        }
        possibleCells = nextClosestCells({x, y}, maxDist, skip);
    }
    return results;
}


void PointGrid::processCellPointsXd(Eigen::VectorXd pos,
    const DistanceResults& possibleCells, KnnResults& results) const
{
    for (const DistanceResult& possibleCell : possibleCells)
    {
        uint32_t k = static_cast<uint32_t>(possibleCell.index);
        const Cell& c = m_cells[k];
        for (PointId id : c)
        {
            Eigen::VectorXd pos2(m_dims.size());
            for (size_t i = 0; i < m_dims.size(); ++i)
                pos2(i) = m_view.getFieldAs<double>(m_dims[i], id);
            results.tryInsert(id, (pos - pos2).squaredNorm());
        }
        // Need to bail here if the results are full and the distance is less than that
        // of the next possibleCell.
    }
}

PointGrid::DistanceResults PointGrid::radiusSearch(Eigen::VectorXd pos, double radius) const
{
    DistanceResults results;
    double radius2 = radius * radius;  // We use square distance
    for (uint32_t key : radiusCells({pos(0), pos(1)}, radius))
        for (PointId id : m_cells[key])
        {
            Eigen::VectorXd pos2(m_dims.size());
            for (size_t i = 0; i < m_dims.size(); ++i)
                pos2(i) = m_view.getFieldAs<double>(m_dims[i], id);
            double dist = (pos - pos2).squaredNorm();
            if (dist < radius2)
                results.emplace_back(id, dist);
        }
    std::sort(results.begin(), results.end());
    return results;
}


/// Internal ///

PointGrid::DistanceResults PointGrid::findCells(Eigen::Vector2d pos, double maxDist,
    std::vector<uint32_t>& skip, BOX2D box) const
{
    DistanceResults cells;

    auto [imin, jmin] = toIJ(box.minx, box.miny);
    auto [imax, jmax] = toIJ(box.maxx, box.maxy);

    for (uint16_t i = imin; i <= imax; ++i)
        for (uint16_t j = jmin; j <= jmax; ++j)
        {
            uint32_t curKey = key(i, j);
            if (std::find(skip.begin(), skip.end(), curKey) != skip.end())
                continue;

            double cellDist = boundsDistance(pos(0), pos(1), bounds(i, j));

            if (cellDist <= maxDist)
            {
                cells.emplace_back(curKey, cellDist);
                skip.push_back(curKey);
            }
        }
    std::sort(cells.begin(), cells.end());
    return cells;
}

PointGrid::DistanceResults PointGrid::nextClosestCells(Eigen::Vector2d pos, double maxDist,
    std::vector<uint32_t>& skip) const
{
    BOX2D box;
    box.grow(pos(0), pos(1));
    box.grow(maxDist);
    box.clip(m_bounds);

    return findCells(pos, maxDist, skip, box);
}

std::vector<uint32_t> PointGrid::radiusCells(Eigen::Vector2d pos,
    const double radius) const
{
    BOX2D box;
    box.grow(pos(0), pos(1));
    box.grow(radius);
    box.clip(bounds());

    // If the bounds of the grid and the box containing the circle with center pos don't
    // overlap, there are no relevant cells.
    std::vector<uint32_t> cells;
    if (!bounds().overlaps(box))
        return cells;

    auto [imin, jmin] = toIJ(box.minx, box.miny);
    auto [imax, jmax] = toIJ(box.maxx, box.maxy);

    cells.reserve((imax - imin + 1) * (jmax - jmin + 1));
    for (uint16_t i = imin; i <= imax; ++i)
        for (uint16_t j = jmin; j <= jmax; ++j)
            cells.push_back(key(i, j));
    return cells;
}


std::vector<uint32_t> PointGrid::nextCells(Eigen::Vector2d pos,
    double maxDistSq, std::vector<uint32_t>& skip) const
{
    std::vector<uint32_t> cells;
    for (uint32_t key : radiusCells(pos, std::sqrt(maxDistSq)))
        if (std::find(skip.begin(), skip.end(), key) == skip.end())
            cells.push_back(key);
    return cells;
}

} // namespace pdal
