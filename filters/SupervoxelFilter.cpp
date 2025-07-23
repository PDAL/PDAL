/******************************************************************************
 * Copyright (c) 2025, Bram Ton (bram@cbbg.nl)
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

// Yangbin Lin, Cheng Wang, Dawei Zhai, Wei Li, Jonathan Li,
// Toward better boundary preserved supervoxel segmentation for 3D point clouds,
// ISPRS Journal of Photogrammetry and Remote Sensing, Volume 143, 2018,
// Pages 39-47, ISSN 0924-2716, doi:10.1016/j.isprsjprs.2018.05.004
//
// This implementation is derived from the work of the original authors:
// https://github.com/yblin/Supervoxel-for-3D-point-clouds

#include "SupervoxelFilter.hpp"

#include "private/DisjointSet.hpp"

#include <pdal/KDIndex.hpp>

#include <limits>
#include <Eigen/Core>


namespace pdal
{

using namespace Eigen;
using namespace Dimension;
using namespace std::chrono;

static StaticPluginInfo const s_info{
    "filters.supervoxel", "Supervoxel segmentation.",
    "http://pdal.io/stages/filters.supervoxel.html"};

CREATE_STATIC_STAGE(SupervoxelFilter, s_info)

std::string SupervoxelFilter::getName() const
{
    return s_info.name;
}

SupervoxelFilter::SupervoxelFilter() : Filter()
{
}

void SupervoxelFilter::addArgs(ProgramArgs& args)
{
    args.add("knn", "k nearest neighbours", m_knn,
             static_cast<uint64_t>(32));
    args.add("resolution", "Resolution", m_R, 1.0);
}

void SupervoxelFilter::addDimensions(PointLayoutPtr layout)
{
    layout->registerDim(Id::ClusterID);
}

void SupervoxelFilter::prepared(PointTableRef table)
{
    const PointLayoutPtr layout(table.layout());
    if (!(layout->hasDim(Id::NormalX)) ||
        !(layout->hasDim(Id::NormalY)) ||
        !(layout->hasDim(Id::NormalZ)))
        throwError("No normals found.");
}

/** Estimate number of supervoxels based on resolution.
  * Code copied from VoxelDownSizeFilter.cpp
  */
size_t SupervoxelFilter::estimateClusterCount(PointView &view)
{
    using Voxel = std::tuple<int, int, int>;
    std::set<Voxel> populatedVoxels;
    PointRef point(view);
    double x,y,z;
    double originX, originY, originZ;

    for (PointId id = 0; id < view.size(); ++id)
    {
        point.setPointId(id);
        x = point.getFieldAs<double>(Dimension::Id::X);
        y = point.getFieldAs<double>(Dimension::Id::Y);
        z = point.getFieldAs<double>(Dimension::Id::Z);

        if (populatedVoxels.empty())
        {
            originX = x - (m_R / 2);
            originY = y - (m_R / 2);
            originZ = z - (m_R / 2);
        }

        // Offset by origin.
        x -= originX;
        y -= originY;
        z -= originZ;

        Voxel v = std::make_tuple((int)(std::floor(x / m_R)),
            (int)(std::floor(y / m_R)), (int)(std::floor(z / m_R)));

        populatedVoxels.insert(v);
    }
    return populatedVoxels.size();
}

double SupervoxelFilter::lambda0(PointView &view, std::vector<PointIdList> &G)
{
   std::vector<double> dists(G.size(), std::numeric_limits<double>::max());
   double d;
   PointRef i_ref(view);
   PointRef j_ref(view);
   for (PointId i = 0; i < G.size(); ++i)
   {
       for (PointId const j : G[i])
       {
           if (i == j) continue;
           i_ref.setPointId(i);
           j_ref.setPointId(j);
           d = dist(i_ref, j_ref);
           dists[i] = std::min(dists[i], d);
       }
   }
   // Determine median value
   std::nth_element(dists.begin(), dists.begin()+dists.size()/2, dists.end());
   return std::max(dists[dists.size()/2],
                   std::numeric_limits<double>::epsilon());
}

double SupervoxelFilter::dist(const PointRef& pi, const PointRef& pj)
{
    Vector3d p1(pi.getFieldAs<double>(Id::X),
                pi.getFieldAs<double>(Id::Y),
                pi.getFieldAs<double>(Id::Z));
    Vector3d p2(pj.getFieldAs<double>(Id::X),
                pj.getFieldAs<double>(Id::Y),
                pj.getFieldAs<double>(Id::Z));
    Vector3d n1(pi.getFieldAs<double>(Id::NormalX),
                pi.getFieldAs<double>(Id::NormalY),
                pi.getFieldAs<double>(Id::NormalZ));
    Vector3d n2(pj.getFieldAs<double>(Id::NormalX),
                pj.getFieldAs<double>(Id::NormalY),
                pj.getFieldAs<double>(Id::NormalZ));
    double d = 1 - abs(n1.dot(n2));
    d += 0.4*(p1 - p2).norm()/m_R;
    return d;
}

void SupervoxelFilter::filter(PointView& view)
{

    // We are done if there's nothing in the view
    if (view.empty())
        return;

    const point_count_t n_points = view.size();
    KD3Index& kdi = view.build3dIndex();
    size_t ncluster = estimateClusterCount(view);
    log()->get(LogLevel::Info) << getName() << ": number of clusters: "
        << ncluster << std::endl;
    DisjointSet djset(n_points);
    std::set<PointId> roots;
    std::vector<bool> visited(n_points, false);
    std::vector<unsigned int> c(n_points, 1);

    // Initialise neighbour list and cluster roots
    std::vector<PointIdList> G(n_points);
    for (PointId idx = 0; idx < n_points; ++idx)
    {
        G[idx] = kdi.neighbors(idx, m_knn);
        roots.insert(idx);
    }

    double lambda = lambda0(view, G);
    log()->get(LogLevel::Debug) << getName() << ": initial lambda value: "
        << lambda << std::endl;

    ////////////////////////////
    // Fusion based minimization
    ////////////////////////////
    unsigned int front, back;
    PointId rj;
    PointRef ri_ref(view);
    PointRef rj_ref(view);
    PointIdList queue(n_points);
    while (roots.size() > ncluster) {
        for (auto ri : roots)
        {
            if (G[ri].empty()) continue;

            front = 0; back = 1;
            visited[ri] = true;
            queue[front++] = ri;
            for (PointId j : G[ri])
            {
                j = djset.find(j);
                if (!visited[j])
                {
                    visited[j] = true;
                    queue[back++] = j;
                }
            }

            PointIdList Gi;
            while (front < back)
            {
                rj = queue[front++];
                ri_ref.setPointId(ri);
                rj_ref.setPointId(rj);
                if ((lambda - c[rj] * dist(ri_ref, rj_ref)) > 0)
                {
                    djset.unite(ri, rj); // Merge rj into ri
                    roots.erase(rj);
                    c[ri] += c[rj];
                    for (PointId k : G[rj])
                    {
                        k = djset.find(k);
                        if (!visited[k])
                        {
                            visited[k] = true;
                            queue[back++] = k;
                        }
                    }
                    G[rj] = PointIdList(); // Empty G[rj]
                    if (roots.size() == ncluster) break;
                } else {
                    Gi.push_back(rj);
                }
            }
            G[ri] = Gi;

            for (unsigned int i = 0; i < back; ++i)
            {
                visited[queue[i]] = false;
            }
            if (roots.size() == ncluster) break;
        }


        if (roots.size() == ncluster) break;

        lambda *= 2.0;
        if (lambda == std::numeric_limits<double>::infinity()) {
            log()->get(LogLevel::Warning) << getName() << ": lambda "
                "reached it limit." << std::endl;
            break;
        }
    }

    // Free some memory
    c = std::vector<unsigned int>();
    queue = PointIdList();

    //////////////////////////////
    // Exchange based minimization
    //////////////////////////////
    std::vector<PointId> labels(n_points);
    std::vector<double> dists(n_points);
    PointRef pi_ref(view);
    for (PointId i = 0; i < n_points; ++i)
    {
        PointId ri = djset.find(i);
        labels[i] = ri;
        if (i != ri)
        {
            pi_ref.setPointId(i);
            ri_ref.setPointId(ri);
            dists[i] = dist(pi_ref, ri_ref);
        } else {
            dists[i] = 0;
        }
        visited[i] = false;
    }

    // Re-initialise neighbour list, as it has been modified in the previous step
    for (PointId idx = 0; idx < n_points; ++idx)
    {
        G[idx] = kdi.neighbors(idx, m_knn);
    }

    // Create queue of supervoxel edge points
    std::queue<PointId> q;
    for (PointId pi = 0; pi < n_points; ++pi)
    {
        for (auto pj: G[pi])
        {
            if (labels[pi] == labels[pj]) continue;
            if (!visited[pi])
            {
                q.push(pi);
                visited[pi] = true;
            }
            if (!visited[pj])
            {
                q.push(pj);
                visited[pj] = true;
            }
        }
    }

    while (!q.empty())
    {
        PointId pi = q.front();
        q.pop();
        visited[pi] = false;
        bool change = false;
        for (auto pj: G[pi])
        {
            PointId ri = labels[pi];
            PointId rj = labels[pj];
            if (ri == rj) continue;
            pi_ref.setPointId(pi);
            rj_ref.setPointId(rj);
            double d = this->dist(pi_ref, rj_ref);
            if (d < dists[pi])
            {
                labels[pi] = rj;
                dists[pi] = d;
                change = true;
            }
        }
        if (change)
        {
            for (auto pj: G[pi])
            {
                if ((labels[pi] != labels[pj]) && (!visited[pj]))
                {
                    q.push(pj);
                    visited[pj] = true;
                }
            }
        }
    }


    ////////////////////////////////////////
    // Map cluster ids to consecutive labels
    ////////////////////////////////////////
    std::map<PointId, int64_t> label_map;
    auto it = roots.begin();
    for (PointId idx = 0; idx < roots.size(); ++idx)
    {
        label_map[*it] = idx;
        it++;
    }

    for (PointId idx = 0; idx < n_points; ++idx)
    {
        int64_t label = label_map[labels[idx]];
        view.setField(Dimension::Id::ClusterID, idx, label);
    }
}

} // namespace pdal
