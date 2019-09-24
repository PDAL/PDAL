/******************************************************************************
 * Copyright (c) 2019, Bradley J Chambers (brad.chambers@gmail.com)
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

// Adapted from MIT-licensed implemenation provided by
// https://github.com/intel-isl/Open3D/pull/1038.

#include "DBSCANFilter.hpp"

#include <pdal/KDIndex.hpp>

#include <string>
#include <unordered_set>

namespace pdal
{

class KDFlexIndex
{
protected:
    const PointView& m_buf;
    const Dimension::IdList& m_dims;

    typedef nanoflann::KDTreeSingleIndexAdaptor<
        nanoflann::L2_Simple_Adaptor<double, KDFlexIndex, double>, KDFlexIndex,
        -1, std::size_t>
        my_kd_tree_t;

    std::unique_ptr<my_kd_tree_t> m_index;

public:
    std::size_t kdtree_get_point_count() const
    {
        return m_buf.size();
    }

    void build()
    {
        m_index.reset(
            new my_kd_tree_t(m_dims.size(), *this,
                             nanoflann::KDTreeSingleIndexAdaptorParams(100)));
        m_index->buildIndex();
    }

    KDFlexIndex(const PointView& buf, const Dimension::IdList& dims)
        : m_buf(buf), m_dims(dims)
    {
    }

    ~KDFlexIndex()
    {
    }

    PointIdList radius(PointId idx, double r) const
    {
        PointIdList output;
        std::vector<std::pair<std::size_t, double>> ret_matches;
        nanoflann::SearchParams params;
        params.sorted = true;

        std::vector<double> pt;
        for (auto const& dim : m_dims)
        {
            double val = m_buf.getFieldAs<double>(dim, idx);
            pt.push_back(val);
        }

        // Our distance metric is square distance, so we use the square of
        // the radius.
        const std::size_t count =
            m_index->radiusSearch(&pt[0], r * r, ret_matches, params);

        for (std::size_t i = 0; i < count; ++i)
            output.push_back(ret_matches[i].first);
        return output;
    }

    inline double kdtree_get_pt(const PointId idx, int dim) const
    {
        if (idx >= m_buf.size())
            return 0.0;

        return m_buf.getFieldAs<double>(m_dims[dim], idx);
    }

    inline double kdtree_distance(const double* p1, const PointId idx,
                                  size_t /*numDims*/) const
    {
        double result(0.0);
        for (size_t i = 0; i < m_dims.size(); ++i)
        {
            double d = p1[i] - m_buf.getFieldAs<double>(m_dims[i], idx);
            result += d * d;
        }

        return result;
    }

    template <class BBOX> bool kdtree_get_bbox(BBOX& bb) const
    {
        if (m_buf.empty())
        {
            for (size_t i = 0; i < m_dims.size(); ++i)
            {
                bb[i].low = 0.0;
                bb[i].high = 0.0;
            }
        }
        else
        {
            for (size_t i = 0; i < m_dims.size(); ++i)
            {
                bb[i].low = std::numeric_limits<double>::max();
                bb[i].high = std::numeric_limits<double>::lowest();
            }

            for (PointId i = 0; i < m_buf.size(); ++i)
            {
                for (size_t j = 0; j < m_dims.size(); ++j)
                {
                    double val = m_buf.getFieldAs<double>(m_dims[j], i);
                    if (val < bb[j].low)
                        bb[j].low = val;
                    if (val > bb[j].high)
                        bb[j].high = val;
                }
            }
        }
        return true;
    }

private:
    KDFlexIndex(const KDFlexIndex&);
    KDFlexIndex& operator=(KDFlexIndex&);
};

static StaticPluginInfo const s_info{
    "filters.dbscan", "DBSCAN Clustering.",
    "http://pdal.io/stages/filters.dbscan.html"};

CREATE_STATIC_STAGE(DBSCANFilter, s_info)

std::string DBSCANFilter::getName() const
{
    return s_info.name;
}

DBSCANFilter::DBSCANFilter() : Filter()
{
}

void DBSCANFilter::addArgs(ProgramArgs& args)
{
    args.add("min_points", "Min points per cluster", m_minPoints,
             static_cast<uint64_t>(6));
    args.add("eps", "Epsilon", m_eps, 1.0);
    args.add("dimensions", "Dimensions to cluster", m_dimStringList);
}

void DBSCANFilter::addDimensions(PointLayoutPtr layout)
{
    m_cluster =
        layout->registerOrAssignDim("ClusterID", Dimension::Type::Signed64);
}

void DBSCANFilter::prepared(PointTableRef table)
{
    const PointLayoutPtr layout(table.layout());

    if (m_dimStringList.size())
    {
        for (std::string& s : m_dimStringList)
        {
            Dimension::Id id = layout->findDim(s);
            if (id == Dimension::Id::Unknown)
                throwError("Invalid dimension '" + s +
                           "' specified for "
                           "'dimensions' option.");
            m_dimIdList.push_back(id);
        }
    }
}

void DBSCANFilter::filter(PointView& view)
{
    // Construct KDFlexIndex for radius search.
    KDFlexIndex kdfi(view, m_dimIdList);
    kdfi.build();

    // First pass through point cloud precomputes neighbor indices and
    // initializes ClusterID to -2.
    std::vector<PointIdList> neighbors(view.size());
    for (PointId idx = 0; idx < view.size(); ++idx)
    {
        neighbors[idx] = kdfi.radius(idx, m_eps);
        view.setField(m_cluster, idx, -2);
    }

    // Second pass through point cloud performs DBSCAN clustering.
    int64_t cluster_label = 0;
    for (PointId idx = 0; idx < view.size(); ++idx)
    {
        // Point has already been labeled, so move on to next.
        if (view.getFieldAs<int64_t>(m_cluster, idx) != -2)
            continue;

        // Density of the neighborhood does not meet minimum number of points
        // constraint, label as noise.
        if (neighbors[idx].size() < m_minPoints)
        {
            view.setField(m_cluster, idx, -1);
            continue;
        }

        // Initialize some bookkeeping to track which neighbors have been
        // considered for addition to the current cluster.
        std::unordered_set<PointId> neighbors_next(neighbors[idx].begin(),
                                                   neighbors[idx].end());
        std::unordered_set<PointId> neighbors_visited;
        neighbors_visited.insert(idx);

        // Unlabeled point encountered; assign cluster label.
        view.setField(m_cluster, idx, cluster_label);

        // Consider all neighbors.
        while (!neighbors_next.empty())
        {
            // Select first neighbor and move it to the visited set.
            PointId p = *neighbors_next.begin();
            neighbors_next.erase(neighbors_next.begin());
            neighbors_visited.insert(p);

            // Reassign cluster label to neighbor previously marked as noise.
            if (view.getFieldAs<int64_t>(m_cluster, p) == -1)
                view.setField(m_cluster, p, cluster_label);

            // Neighbor has already been labeled, so move on to next.
            if (view.getFieldAs<int64_t>(m_cluster, p) != -2)
                continue;

            // Assign cluster label to neighbor.
            view.setField(m_cluster, p, cluster_label);

            // If density of neighbor's neighborhood is sufficient, add it's
            // neighbors to the set of neighbors to consider if they are not
            // already.
            if (neighbors[p].size() >= m_minPoints)
            {
                for (PointId q : neighbors[p])
                {
                    if (neighbors_visited.count(q) == 0)
                        neighbors_next.insert(q);
                }
            }
        }

        cluster_label++;
    }
}

} // namespace pdal
