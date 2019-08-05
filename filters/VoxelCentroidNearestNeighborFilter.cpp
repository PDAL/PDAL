/******************************************************************************
 * Copyright (c) 2017, Bradley J Chambers (brad.chambers@gmail.com)
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

#include <pdal/EigenUtils.hpp>
#include <pdal/KDIndex.hpp>

#include <map>
#include <string>
#include <tuple>
#include <vector>

#include <Eigen/Dense>

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
    BOX3D bounds;
    calculateBounds(*view, bounds);

    KD3Index& kdi = view->build3dIndex();

    // Make an initial pass through the input PointView to index PointIds by
    // row, column, and depth.
    std::map<std::tuple<size_t, size_t, size_t>, std::vector<PointId>>
        populated_voxel_ids;
    for (PointId id = 0; id < view->size(); ++id)
    {
        double y = view->getFieldAs<double>(Dimension::Id::Y, id);
        double x = view->getFieldAs<double>(Dimension::Id::X, id);
        double z = view->getFieldAs<double>(Dimension::Id::Z, id);
        size_t r = static_cast<size_t>((y - bounds.miny) / m_cell);
        size_t c = static_cast<size_t>((x - bounds.minx) / m_cell);
        size_t d = static_cast<size_t>((z - bounds.minz) / m_cell);
        populated_voxel_ids[std::make_tuple(r, c, d)].push_back(id);
    }

    // Make a second pass through the populated voxels to compute the voxel
    // centroid and to find its nearest neighbor.
    PointViewPtr output = view->makeNew();
    for (auto const& t : populated_voxel_ids)
    {
        Eigen::Vector3d centroid = computeCentroid(*view, t.second);
        std::vector<PointId> neighbors =
            kdi.neighbors(centroid[0], centroid[1], centroid[2], 1);
        output->appendPoint(*view, neighbors[0]);
    }

    PointViewSet viewSet;
    viewSet.insert(output);
    return viewSet;
}

} // namespace pdal
