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


/// 2D ///


/*

DistanceResults PointGrid::knnSearch(Eigen::Matrix<double, Eigen::Dynamic, 1> pos, point_count_t k) const
{
    // throw instead?
    if (pos.rows() < 2 || pos.rows() > 3)
        return;

    DistanceResults results;
    results.reserve(k);

    // Find the starting cell
    auto [iStart, jStart] = toIJ(pos(0), pos(1));
    if (k > m_view.size()) // edge case
        k = m_view.size();

    // Maximum possible distance between 2 points. A bit extreme but it's the most accurate.
    // Maybe should be different for 2d and 3d
    const double maxSqDistance = std::pow(m_bounds.maxx - m_bounds.minx, 2) +
        std::pow(m_bounds.maxy - m_bounds.miny, 2) + std::pow(m_bounds.maxz - m_bounds.minz, 2);
    // Setting this arbitrarily large for the first iteration
    double curMaxDist = maxSqDistance;
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
                Eigen::Matrix<double, pos.rows(), 1> pos2or3;
                //!! is this construction efficient?
                pos2or3 << m_view.getFieldAs<double>(Dimension::Id::X, id),
                    m_view.getFieldAs<double>(Dimension::Id::Y, id);
                if (pos2or3.rows() == 3)
                    pos2or3 << m_view.getFieldAs<double>(Dimension::Id::Z, id);

                double dist2 = (pos2or3 - pos).squaredNorm();
                if (dist2 < curMaxDist)
                    results.emplace_back(id, dist2);
            }
            // Sort by distance
            std::sort(results.begin(), results.end(), [](const std::pair<PointId, double>& a,
                const std::pair<PointId, double>& b) {
                return a.second < b.second;
            });
            // If we've found enough, we can use a more precise max distance as our radius
            // in nextCells(). If not, use the one that was set at the start (assuming this is the
            // first iteration).
            if (results.size() >= k) // resize is dumb if size == k but whatever
            {
                results.resize(k);
                //!! Shouldn't need the min check. Remove
                curMaxDist = results.back().second;
            }
            // If it's not the first iteration and still fails to find k, grow the radius
            // by a bit. If we already tried and we reached the max distance, we're done
            else if (skip.size() > 1)
            {
                if (curMaxDist == maxSqDistance)
                    break;
                curMaxDist = std::min((curMaxDist * 1.5), maxSqDistance);
                std::cout << "key " << key << ": Growing radius to " << curMaxDist << std::endl;
            }
        }
        // Get the next cells. If there are none, we're done
        keys = nextCells(pos, curMaxDist, skip);
    }
    //!! Warn if we didn't find enough?
    return results;
}
*/

PointIdList PointGrid::neighbors(double x, double y, point_count_t k) const
{
    DistanceResults results = knnSearch(x, y, k);
    PointIdList pts;
    for (auto& r : results)
        pts.push_back(r.first);

    return pts;
}

// Always potentially bounded by maxSqDistance, which can be set to an arbitrararily large value
// to include all points.
DistanceResults PointGrid::knnSearch(double x, double y, point_count_t k) const
{
    Eigen::Vector2d pos(x, y);
    DistanceResults results;
    results.reserve(k);

    // Find the starting cell
    auto [iStart, jStart] = toIJ(x, y);
    if (k > m_view.size()) // edge case
        k = m_view.size();

    // Maximum possible distance between 2 points.
    const double maxSqDistance = std::pow(m_bounds.maxx - m_bounds.minx, 2) +
        std::pow(m_bounds.maxy - m_bounds.miny, 2);
    // Starting off as the diagonal of a single cell.
    double curMaxDist = m_xlen * m_xlen + m_ylen * m_ylen;
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
            std::sort(results.begin(), results.end(), [](const std::pair<PointId, double>& a,
                const std::pair<PointId, double>& b) {
                return a.second < b.second;
            });
            // If we've found enough, we can use a more precise max distance as our radius
            // in nextCells(). If not, use the one that was set at the start (assuming this is the
            // first iteration).
            if (results.size() >= k) // resize is dumb if size == k but whatever
            {
                results.resize(k);
                curMaxDist = results.back().second;
            }
            // If it's not the first iteration and still fails to find k, grow the radius
            // by a bit. If we already tried and we reached the max distance, we're done
            else if (skip.size() > 1)
            {
                if (curMaxDist == maxSqDistance)
                    break;
                curMaxDist = (std::min)((curMaxDist * 1.5), maxSqDistance);
                std::cout << "key " << key << ": Growing radius to " << curMaxDist << std::endl;
            }
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
        neighbors.push_back(results[i].first);
    return neighbors;
}


DistanceResults PointGrid::radiusSearch(double x, double y, double radius) const
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

    std::sort(results.begin(), results.end(), [](const std::pair<PointId, double>& a,
        const std::pair<PointId, double>& b) {
        return a.second < b.second;
    });

    return results;
}


/// 3D ///


// 3D version of knnSearch. Should consolidate if possible
DistanceResults PointGrid::knnSearch(double x, double y, double z, point_count_t k) const
{
    Eigen::Vector3d pos(x, y, z);
    DistanceResults results;
    results.reserve(k);

    // Find the starting cell
    auto [iStart, jStart] = toIJ(x, y);
    if (k > m_view.size()) // edge case
        k = m_view.size();

    // Maximum possible distance between 2 points
    const double maxSqDistance = std::pow(m_bounds.maxx - m_bounds.minx, 2) +
        std::pow(m_bounds.maxy - m_bounds.miny, 2) + std::pow(m_bounds.maxz - m_bounds.minz, 2);
    //!! Compared to 2D, this could expand our queue by too much if < k neighbors since the
    //!! nextCells() search is 2D. Could remove the Z padding later but I need to be careful
    //!! (plus it doesn't matter that much)
    // Starting off as the diagonal of a single cell, with some padding in the Z direction
    // for the first iteration.
    double curMaxDist = m_xlen * m_xlen + m_ylen * m_ylen +
        std::pow(m_bounds.maxz - m_bounds.minz, 2);
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
                Eigen::Vector3d pos3(m_view.getFieldAs<double>(Dimension::Id::X, id),
                    m_view.getFieldAs<double>(Dimension::Id::Y, id),
                    m_view.getFieldAs<double>(Dimension::Id::Z, id));
                double dist2 = (pos3 - pos).squaredNorm();
                if (dist2 < curMaxDist)
                    results.emplace_back(id, dist2);
            }
            // Sort by distance
            std::sort(results.begin(), results.end(), [](const std::pair<PointId, double>& a,
                const std::pair<PointId, double>& b) {
                return a.second < b.second;
            });
            // If we've found enough, we can use a more precise max distance as our radius
            // in nextCells(). If not, use the one that was set at the start (assuming this is the
            // first iteration).
            if (results.size() >= k) // resize is dumb if size == k but whatever
            {
                results.resize(k);
                curMaxDist = results.back().second;
            }
            // If it's not the first iteration and still fails to find k, grow the radius
            // by a bit. If we already tried and we reached the max distance, we're done
            else if (skip.size() > 1)
            {
                if (curMaxDist == maxSqDistance)
                    break;
                curMaxDist = (std::min)((curMaxDist * 1.5), maxSqDistance);
                std::cout << "key " << key << ": Growing radius to " << curMaxDist << std::endl;
            }
        }
        // Get the next cells. If there are none, we're done
        keys = nextCells({pos(0), pos(1)}, curMaxDist, skip);
    }
    //!! Warn if we didn't find enough?
    return results;
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
        output[i] = results[i * stride].first;

    return output;
}


PointIdList PointGrid::radius(double x, double y, double z, double radius) const
{
    PointIdList neighbors;
    DistanceResults results = radiusSearch(x, y, z, radius);
    for (size_t i = 0; i < results.size(); ++i)
        neighbors.push_back(results[i].first);
    return neighbors;
}


DistanceResults PointGrid::radiusSearch(double x, double y, double z, double radius) const
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

    std::sort(results.begin(), results.end(), [](const std::pair<PointId, double>& a,
        const std::pair<PointId, double>& b) {
        return a.second < b.second;
    });

    return results;
}

/// Internal ///


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

/*
// don't know if this needs to exist.
PointIdList PointGrid::boxEncloses(BOX3D extent) const
{
    BOX3D boundsExtent(bounds());
    boundsExtent.minz = extent.minz;
    boundsExtent.maxz = extent.maxz;
    extent.clip(boundsExtent);

    PointIdList neighbors;

    // Find IJ bounding box.
    auto [imin, jmin] = toIJ(extent.minx, extent.miny);
    auto [imax, jmax] = toIJ(extent.maxx, extent.maxy);

    for (uint16_t i = imin; i <= imax; ++i)
        for (uint16_t j = jmin; j <= jmax; ++j)
        {
            const Cell& c = cell(i, j);

            // Check every point to make sure it's in the extent.
            for (PointId id : c)
            {
                double x = m_view.getFieldAs<double>(Dimension::Id::X, id);
                double y = m_view.getFieldAs<double>(Dimension::Id::Y, id);
                double z = m_view.getFieldAs<double>(Dimension::Id::Z, id);
                if (extent.contains(x, y, z))
                    neighbors.push_back(id);
            }
        }
    return neighbors;
}
*/

} // namespace pdal
