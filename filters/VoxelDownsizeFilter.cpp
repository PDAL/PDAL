/******************************************************************************
 * Copyright (c) 2019, Helix.re
 * Contact Person : Pravin Shinde (pravin@helix.re,
 *                    https://github.com/pravinshinde825)
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

#include "VoxelDownsizeFilter.hpp"

namespace pdal
{

static StaticPluginInfo const s_info
{
    "filters.voxeldownsize",
    "First Entry Voxel Filter", 
    "http://pdal.io/stages/filters.voxeldownsize.html"
};

CREATE_STATIC_STAGE(VoxelDownsizeFilter, s_info)

VoxelDownsizeFilter::VoxelDownsizeFilter()
{}


std::string VoxelDownsizeFilter::getName() const
{
    return s_info.name;
}

void VoxelDownsizeFilter::addArgs(ProgramArgs& args)
{
    args.add("cell", "Cell size", m_cell, 0.001);
    args.add("mode", "Method for downsizing : voxelcenter / firstinvoxel",
        m_mode, "voxelcenter");
}


void VoxelDownsizeFilter::ready(PointTableRef)
{
    m_pivotVoxelInitialized = false;
}


void VoxelDownsizeFilter::prepared(PointTableRef)
{
    m_mode = Utils::toupper(m_mode);
    if (m_mode == "VOXELCENTER")
        m_isFirstInVoxelMode = false;
    else if (m_mode == "FIRSTINVOXEL")
        m_isFirstInVoxelMode = true;
    else
        throwError("Invalid mode specified.");
}


PointViewSet VoxelDownsizeFilter::run(PointViewPtr view)
{
    PointViewPtr output = view->makeNew();
    for (PointId id = 0; id < view->size(); ++id)
    {
        if (voxelize(view->point(id)))
        {
            output->appendPoint(*view, id);
        }
    }

    PointViewSet viewSet;
    viewSet.insert(output);
    return viewSet;
}


bool VoxelDownsizeFilter::voxelize(PointRef point)
{
    /*
     * Calculate the voxel coordinates for the incoming point.
     * gx, gy, gz will be the global coordinates from (0, 0, 0).
     */
    int gx = std::floor(point.getFieldAs<double>(Dimension::Id::X) / m_cell);
    int gy = std::floor(point.getFieldAs<double>(Dimension::Id::Y) / m_cell);
    int gz = std::floor(point.getFieldAs<double>(Dimension::Id::Z) / m_cell);

    if (!m_pivotVoxelInitialized)
    {
        /*
         * Save global coordinates of first incoming point's voxel.
         * This will act as a Pivot for calculation of local coordinates of the
         * voxels.
         */
        m_pivotVoxel[0] = gx; // X Coordinate of an Pivot voxel
        m_pivotVoxel[1] = gy; // Y Coordinate of an Pivot voxel
        m_pivotVoxel[2] = gz; // Z Coordinate of an Pivot voxel
        m_pivotVoxelInitialized = true;
    }

    /*
     * Calculate the local voxel coordinates for incoming point, Using the
     * Pivot voxel.
     */
    auto t = std::make_tuple(gx - m_pivotVoxel[0], gy - m_pivotVoxel[1],
                             gz - m_pivotVoxel[2]);

    auto inserted = m_populatedVoxels.insert(t).second;
    if (!m_isFirstInVoxelMode && inserted)
    {
        point.setField<double>(Dimension::Id::X, (gx + 0.5) * m_cell);
        point.setField<double>(Dimension::Id::Y, (gy + 0.5) * m_cell);
        point.setField<double>(Dimension::Id::Z, (gz + 0.5) * m_cell);
    }
    return inserted;
}

bool VoxelDownsizeFilter::processOne(PointRef& point)
{
    return voxelize(point);
}

} // namespace pdal
