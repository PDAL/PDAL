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

#include "VoxelCentroidNearestNeighborFilter.hpp"

#include <map>
#include <string>
#include <tuple>
#include <vector>

#include <Eigen/Dense>

#include <pdal/private/MathUtils.hpp>

namespace pdal
{

static StaticPluginInfo const s_info
{
    "filters.voxelcentroidnearestneighbor",
    "Voxel Centroid Nearest Neighbor Filter",
    "http://pdal.io/stages/filters.voxelcentroidnearestneighbor.html"
};

CREATE_STATIC_STAGE(VoxelCentroidNearestNeighborFilter, s_info)

std::string VoxelCentroidNearestNeighborFilter::getName() const
{
    return s_info.name;
}

void VoxelCentroidNearestNeighborFilter::addArgs(ProgramArgs& args)
{
    args.add("cell", "Cell size", m_cell, 1.0);
}

PointViewSet VoxelCentroidNearestNeighborFilter::run(PointViewPtr view)
{
    double x0 = view->getFieldAs<double>(Dimension::Id::X, 0);
    double y0 = view->getFieldAs<double>(Dimension::Id::Y, 0);
    double z0 = view->getFieldAs<double>(Dimension::Id::Z, 0);

    // Make an initial pass through the input PointView to index PointIds by
    // row, column, and depth.
    std::map<std::tuple<size_t, size_t, size_t>, PointIdList>
        populated_voxel_ids;
    for (PointId id = 0; id < view->size(); ++id)
    {
        double y = view->getFieldAs<double>(Dimension::Id::Y, id);
        double x = view->getFieldAs<double>(Dimension::Id::X, id);
        double z = view->getFieldAs<double>(Dimension::Id::Z, id);
        size_t r = static_cast<size_t>((y - y0) / m_cell);
        size_t c = static_cast<size_t>((x - x0) / m_cell);
        size_t d = static_cast<size_t>((z - z0) / m_cell);
        populated_voxel_ids[std::make_tuple(r, c, d)].push_back(id);
    }

    // Make a second pass through the populated voxels to compute the voxel
    // centroid and to find its nearest neighbor.
    PointViewPtr output = view->makeNew();
    for (auto const& t : populated_voxel_ids)
    {
        if (t.second.size() == 1)
        {
            // If there is only one point in the voxel, simply append it.
            output->appendPoint(*view, t.second[0]);
        }
        else if (t.second.size() == 2)
        {
            // Else if there are only two, they are equidistant to the
            // centroid, so append the one closest to voxel center.

            // Compute voxel center.
            double y_center = y0 + (std::get<0>(t.first) + 0.5) * m_cell;
            double x_center = x0 + (std::get<1>(t.first) + 0.5) * m_cell;
            double z_center = z0 + (std::get<2>(t.first) + 0.5) * m_cell;

            // Compute distance from first point to voxel center.
            double x1 = view->getFieldAs<double>(Dimension::Id::X, t.second[0]);
            double y1 = view->getFieldAs<double>(Dimension::Id::Y, t.second[0]);
            double z1 = view->getFieldAs<double>(Dimension::Id::Z, t.second[0]);
            double d1 = pow(x_center - x1, 2) + pow(y_center - y1, 2) + pow(z_center - z1, 2);

            // Compute distance from second point to voxel center.
            double x2 = view->getFieldAs<double>(Dimension::Id::X, t.second[1]);
            double y2 = view->getFieldAs<double>(Dimension::Id::Y, t.second[1]);
            double z2 = view->getFieldAs<double>(Dimension::Id::Z, t.second[1]);
            double d2 = pow(x_center - x2, 2) + pow(y_center - y2, 2) + pow(z_center - z2, 2);

            // Append the closer of the two.
            if (d1 < d2)
                output->appendPoint(*view, t.second[0]);
            else
                output->appendPoint(*view, t.second[1]);
        }
        else
        {
            // Else there are more than two neighbors, so choose the one
            // closest to the centroid.

            // Compute the centroid.
            Eigen::Vector3d centroid = math::computeCentroid(*view, t.second);

            // Compute distance from each point in the voxel to the centroid,
            // retaining only the closest.
            PointId pmin = 0;
            double dmin((std::numeric_limits<double>::max)());
            for (auto const& p : t.second)
            {
                double x = view->getFieldAs<double>(Dimension::Id::X, p);
                double y = view->getFieldAs<double>(Dimension::Id::Y, p);
                double z = view->getFieldAs<double>(Dimension::Id::Z, p);
                double sqr_dist = pow(centroid.x() - x, 2) +
                                  pow(centroid.y() - y, 2) +
                                  pow(centroid.z() - z, 2);
                if (sqr_dist < dmin)
                {
                    dmin = sqr_dist;
                    pmin = p;
                }
            }
            output->appendPoint(*view, pmin);
        }
    }

    PointViewSet viewSet;
    viewSet.insert(output);
    return viewSet;
}

} // namespace pdal
