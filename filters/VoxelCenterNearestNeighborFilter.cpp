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

#include "VoxelCenterNearestNeighborFilter.hpp"

#include <pdal/KDIndex.hpp>

#include <set>
#include <string>
#include <tuple>
#include <vector>

namespace pdal
{

static StaticPluginInfo const s_info
{
    "filters.voxelcenternearestneighbor",
    "Voxel Center Nearest Neighbor Filter",
    "http://pdal.io/stages/filters.voxelcenternearestneighbor.html"
};

CREATE_STATIC_STAGE(VoxelCenterNearestNeighborFilter, s_info)

std::string VoxelCenterNearestNeighborFilter::getName() const
{
    return s_info.name;
}

void VoxelCenterNearestNeighborFilter::addArgs(ProgramArgs& args)
{
    args.add("cell", "Cell size", m_cell, 1.0);
}

PointViewSet VoxelCenterNearestNeighborFilter::run(PointViewPtr view)
{
    BOX3D bounds;
    view->calculateBounds(bounds);

    // Find distance from voxel center to point.  If the distance is less
    // than previous (or is the first one for the voxel), store the
    // point ID and distance.
    std::map<std::tuple<size_t, size_t, size_t>,
        std::tuple<PointId, double>> populated_voxels;
    for (PointId id = 0; id < view->size(); ++id)
    {
        double x = view->getFieldAs<double>(Dimension::Id::X, id);
        double y = view->getFieldAs<double>(Dimension::Id::Y, id);
        double z = view->getFieldAs<double>(Dimension::Id::Z, id);
        size_t c = static_cast<size_t>((x - bounds.minx) / m_cell);
        size_t r = static_cast<size_t>((y - bounds.miny) / m_cell);
        size_t d = static_cast<size_t>((z - bounds.minz) / m_cell);
        double xv = bounds.minx + (c + 0.5) * m_cell;
        double yv = bounds.miny + (r + 0.5) * m_cell;
        double zv = bounds.minz + (d + 0.5) * m_cell;
        double dist = pow(xv - x, 2) + pow(yv - y, 2) + pow(zv - z, 2);

        auto t = std::make_tuple(r, c, d);
        auto pi = populated_voxels.find(t);
        if (pi == populated_voxels.end())
            populated_voxels.insert(
                std::make_pair(t, std::make_tuple(id, dist)));
        else
        {
            auto& t2 = pi->second;  // Get point/distance tuple.
            double curDist = std::get<1>(t2);
            if (dist < curDist)
                t2 = std::make_tuple(id, dist);
        }
    }

    // Append the ID of the point nearest the voxel center to the output view.
    PointViewPtr output = view->makeNew();
    for (auto const& t : populated_voxels)
    {
        auto& t2 = t.second; // Get point/distance tuple.
        output->appendPoint(*view, std::get<0>(t2));
    }

    PointViewSet viewSet;
    viewSet.insert(output);
    return viewSet;
}

} // namespace pdal
