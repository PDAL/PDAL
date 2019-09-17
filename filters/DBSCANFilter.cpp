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

static StaticPluginInfo const s_info{
    "filters.dbscan", "DBSCAN Clustering.",
    "http://pdal.io/stages/filters.dbscan.html"};

CREATE_STATIC_STAGE(DBSCANFilter, s_info)

std::string DBSCANFilter::getName() const
{
    return s_info.name;
}

void DBSCANFilter::addArgs(ProgramArgs& args)
{
    args.add("min_points", "Min points per cluster", m_minPoints,
             static_cast<uint64_t>(6));
    args.add("eps", "Epsilon", m_eps, 1.0);
}

void DBSCANFilter::addDimensions(PointLayoutPtr layout)
{
    m_cluster =
        layout->registerOrAssignDim("ClusterID", Dimension::Type::Signed64);
}

void DBSCANFilter::filter(PointView& view)
{
    // Construct 3D KDIndex for radius search.
    KD3Index& kdi = view.build3dIndex();

    // First pass through point cloud precomputes neighbor indices and
    // initializes ClusterID to -2.
    std::vector<std::vector<PointId>> neighbors(view.size());
    for (PointId idx = 0; idx < view.size(); ++idx)
    {
        neighbors[idx] = kdi.radius(idx, m_eps);
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
