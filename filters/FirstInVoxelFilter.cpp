/******************************************************************************
 * Copyright (c) 2019, Helix.re
 * Contact Person : Pravin Shinde (pravin@helix.re, https://github.com/pravinshinde825)
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

static StaticPluginInfo const s_info{
    "filters.first-in-voxel",
    "First Entry Voxel Filter",
    ""};

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
    double gx = point.getFieldAs<double>(Dimension::Id::X) / m_cell;
    double gy = point.getFieldAs<double>(Dimension::Id::Y) / m_cell;
    double gz = point.getFieldAs<double>(Dimension::Id::Z) / m_cell;

    m_Bounds.grow(gx, gy, gz);

    size_t const h1(std::hash<int>{}(static_cast<uint32_t>(POSITIVE_VALUE((gx - m_Bounds.minx)))));
    size_t const h2(std::hash<int>{}(static_cast<uint32_t>(POSITIVE_VALUE((gy - m_Bounds.miny)))));
    size_t const h3(std::hash<int>{}(static_cast<uint32_t>(POSITIVE_VALUE((gz - m_Bounds.minz)))));
    size_t key = std::hash<size_t>{}(HASH_COMBINE((HASH_COMBINE(h1, h2)), h3));

    auto pi = m_BufferMap.find(key);
    if (pi == m_BufferMap.end())
    {
        m_BufferMap.insert(key);
        return true;
    }

    return false;
}

bool FirstInVoxelFilter::processOne(PointRef& point)
{
    return voxelize(point);
}

} // namespace pdal
