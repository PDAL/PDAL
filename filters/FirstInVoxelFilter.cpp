/******************************************************************************
 * Copyright (c) 2019, Helix.re
 * Contact Person : Pravin Shinde (pravin@helix.re,
 *https://github.com/pravinshinde825)
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

#include "FirstInVoxelFilter.hpp"

namespace pdal
{

static StaticPluginInfo const s_info{"filters.first-in-voxel",
                                     "First Entry Voxel Filter", ""};

CREATE_STATIC_STAGE(FirstInVoxelFilter, s_info)

std::string FirstInVoxelFilter::getName() const
{
    return s_info.name;
}

void FirstInVoxelFilter::addArgs(ProgramArgs& args)
{
    args.add("cell", "Cell size", m_cell).setPositional();
}

PointViewSet FirstInVoxelFilter::run(PointViewPtr view)
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

bool FirstInVoxelFilter::voxelize(const PointRef point)
{
    /*
     * Calculate the voxel coordinates for the incoming point.
     * gx, gy, gz will be the global coordinates from (0, 0, 0).
     */
    double gx = point.getFieldAs<double>(Dimension::Id::X) / m_cell;
    double gy = point.getFieldAs<double>(Dimension::Id::Y) / m_cell;
    double gz = point.getFieldAs<double>(Dimension::Id::Z) / m_cell;

    static bool initialized = false;
    if (!initialized)
    {
        /*
         * Save global coordinates of first incoming point.
         * This will act as a Pivot for calculation of local coordinates of the
         * voxels.
         */
        m_pivotVoxel[0] = gx; // X Coordinate of an Pivot voxel
        m_pivotVoxel[1] = gy; // Y Coordinate of an Pivot voxel
        m_pivotVoxel[2] = gz; // Z Coordinate of an Pivot voxel
        initialized = true;
    }

    /*
     * Calculate the local voxel coordinates for incoming point, Using the Pivot
     * voxel. Generate a combined hash key from local coordinates.
     */
    size_t const hx(std::hash<int>{}(std::floor(gx - m_pivotVoxel[0])));
    size_t const hy(std::hash<int>{}(std::floor(gy - m_pivotVoxel[1])));
    size_t const hz(std::hash<int>{}(std::floor(gz - m_pivotVoxel[2])));
    size_t key = std::hash<size_t>{}(hx ^ (hy << 1) ^ (hz << 1));

    /*
     * Is hash key already in populates voxels?
     */
    auto pi = m_populatedVoxels.find(key);
    if (pi == m_populatedVoxels.end())
    {
        /*
         * No, Key is not there in populated voxels.
         * Mark the voxel as populated by making entry in m_populatedVoxels.
         * Accept this point.
         */
        m_populatedVoxels.insert(key);
        return true;
    }
    /*
     * Yes, Key is already in populated voxel keys.
     * That means the voxel is already contains a point.
     * Ignore this point.
     */
    return false;
}

bool FirstInVoxelFilter::processOne(PointRef& point)
{
    return voxelize(point);
}

} // namespace pdal
