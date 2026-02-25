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

#pragma once

#include <optional>
#include <queue>

#include <Eigen/Dense>

#include <pdal/PointView.hpp>
#include <pdal/util/Bounds.hpp>

namespace pdal
{

class PointGrid
{
    using Cell = std::vector<PointId>;
    using OptionalKey = std::optional<uint32_t>;

public:
    struct DistanceResult
    {
        PointId index = 0;
        double sqr_dist = 0.0;

        DistanceResult() = default;
        DistanceResult(PointId index, double sqr_dist) : index(index), sqr_dist(sqr_dist)
        {}

        bool operator<(const DistanceResult& other) const
        { return sqr_dist < other.sqr_dist; }
    };

    using DistanceResults = std::vector<DistanceResult>;

    // Put this somewhere else and probably rename
    class KnnResults
    {
    public:
        KnnResults(size_t knn)
        {
            for (size_t i = 0; i < knn; ++i)
                m_results.push({0, std::numeric_limits<double>::max()});
        }

        void tryInsert(PointId index, double dist)
        {
            m_validCount++;
            if (dist > m_results.top().sqr_dist)
                return;

            m_results.pop();
            m_results.push({index, dist});
        }

        double maxDistance() const
        {
            return m_results.top().sqr_dist;
        }

        size_t validCount() const
        {
            return m_validCount;
        }

        size_t size() const
        {
            return (std::min)(m_results.size(), m_validCount);
        }

        bool full() const
        {
            return m_validCount >= m_results.size();
        }

        std::vector<DistanceResult> sortedResults()
        {
            std::vector<DistanceResult> out(m_results.size());

            size_t pos = m_results.size() - 1;
            while (!m_results.empty())
            {
                out[pos--] = m_results.top();
                m_results.pop();
            }
            return out;
        }

        void sortedResults(std::vector<PointId>& ids, std::vector<double>& dists)
        {
            ids.resize(m_results.size());
            dists.resize(m_results.size());
            size_t pos = m_results.size() - 1;
            while (!m_results.empty())
            {
                ids[pos] = m_results.top().index;
                dists[pos] = m_results.top().sqr_dist;
                m_results.pop();
            }
        }
    private:
        std::priority_queue<DistanceResult> m_results;
        std::size_t m_validCount = 0;
    };

    PointGrid(BOX2D bounds, const PointView& view, int approxPerCell = 200) :
        m_bounds(bounds), m_view(view), m_approxPerCell(approxPerCell)
    {}

    PointGrid(const PointView& view, int approxPerCell = 200) :
        m_view(view), m_approxPerCell(approxPerCell)
    {
        m_view.calculateBounds(m_bounds);
    }

    void build()
    {
        if (m_cells.size())
            return;
        init();
        for (PointId i = 0; i < m_view.size(); ++i)
            add(m_view.getFieldAs<double>(Dimension::Id::X, i),
                m_view.getFieldAs<double>(Dimension::Id::Y, i), i);
    }

    const BOX2D bounds() const
    {
        return m_bounds;
    }

    const PointView& view() const
    {
        return m_view;
    }

    // 2D
    DistanceResults knnSearch(double x, double y, point_count_t k) const;
    PointIdList boxEncloses(BOX2D extent) const;
    PointIdList radius(double x, double y, double radius) const;
    DistanceResults radiusSearch(double x, double y, double radius) const;

    // 3D
    PointIdList radius(double x, double y, double z, double radius) const;
    DistanceResults radiusSearch(double x, double y, double z, double radius) const;
    PointIdList neighbors(double x, double y, double z, point_count_t k,
        int stride) const;
    DistanceResults knnSearch(double x, double y, double z, point_count_t k) const;

private:
    void init()
    {
        // Silently accepting these. Stuff breaks if we don't
        if (m_approxPerCell > (int)m_view.size())
            m_approxPerCell = m_view.size();
        // If we only have one point, grow our bounds by an arbitrary amount since we
        // need to use the bounds to determine search distance, etc.
        if (m_view.size() == 1)
            m_bounds.grow(1.0);
        double cells = std::floor(std::sqrt(m_view.size() / m_approxPerCell));
        assert(cells > 0);
        assert(cells < std::numeric_limits<uint16_t>::max());
        m_cells1d = static_cast<uint16_t>(cells);
        // Adding a small amount to make sure the max value is in a cell.
        m_xlen = (m_bounds.maxx - m_bounds.minx) / m_cells1d + .0001;
        m_ylen = (m_bounds.maxy - m_bounds.miny) / m_cells1d + .0001;
        m_cells.resize(m_cells1d * m_cells1d);
    }

    void add(double x, double y, PointId id)
    {
        auto [i, j] = toIJ(x, y);

        Cell& c = m_cells[key(i, j)];
        if (c.empty())
            c.reserve(m_approxPerCell);
        c.push_back(id);
    }

    // this could be a util function outside this class somewhere.
    double boundsDistanceSq(double x, double y, BOX2D bounds) const
    {
        double xDist = x - std::clamp(x, bounds.minx, bounds.maxx);
        double yDist = y - std::clamp(y, bounds.miny, bounds.maxy);
        return xDist * xDist + yDist * yDist;
    }

    uint32_t key(uint16_t xi, uint16_t yi) const
    {
        return xi * m_cells1d + yi;
    }

    std::pair<uint16_t, uint16_t> toIJ(double x, double y) const
    {
        uint16_t xi = static_cast<uint16_t>((x - m_bounds.minx) / m_xlen);
        uint16_t yi = static_cast<uint16_t>((y - m_bounds.miny) / m_ylen);
        return { xi, yi };
    }

    std::pair<uint16_t, uint16_t> toIJBounded(double x, double y) const
    {
        double xStart = (x - m_bounds.minx) / m_xlen;
        int xIndex = std::clamp(static_cast<int>(xStart), 0, m_cells1d - 1);
        double yStart = (y - m_bounds.miny) / m_ylen;
        int yIndex = std::clamp(static_cast<int>(yStart), 0, m_cells1d - 1);
        return { static_cast<uint16_t>(xIndex), static_cast<uint16_t>(yIndex) };
    }

    const BOX2D bounds(int i, int j) const
    {
        return BOX2D(m_bounds.minx + m_xlen * i, m_bounds.miny + m_ylen * j,
            m_bounds.minx + m_xlen * (i + 1), m_bounds.miny + m_ylen * (j + 1));
    }

    const Cell& cell(uint16_t i, uint16_t j) const
    {
        return m_cells[key(i, j)];
    }

    std::vector<uint32_t> radiusCells(Eigen::Vector2d pos, double radius) const;
    std::vector<uint32_t> nextCells(Eigen::Vector2d pos, double maxDist,
        std::vector<uint32_t>& skip) const;
    OptionalKey nextClosestCell(Eigen::Vector2d pos, double maxDistSq,
        std::vector<uint32_t>& skip) const;
    OptionalKey findCell(Eigen::Vector2d pos, double maxDistSq,
        std::vector<uint32_t>& skip, BOX2D box) const;

    std::vector<Cell> m_cells;
    BOX2D m_bounds;
    const PointView& m_view;
    int m_approxPerCell;
    uint16_t m_cells1d;
    double m_xlen;
    double m_ylen;
};

} // namespace pdal
