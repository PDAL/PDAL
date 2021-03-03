/******************************************************************************
 * Copyright (c) 2016-2017, 2020 Bradley J Chambers (brad.chambers@gmail.com)
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

#include "SampleFilter.hpp"

#include <pdal/util/ProgramArgs.hpp>

#include <string>

namespace pdal
{

using namespace Dimension;

static StaticPluginInfo const s_info{
    "filters.sample", "Subsampling filter",
    "http://pdal.io/stages/filters.sample.html"};

CREATE_STATIC_STAGE(SampleFilter, s_info)

std::string SampleFilter::getName() const
{
    return s_info.name;
}

void SampleFilter::addArgs(ProgramArgs& args)
{
    m_cellArg = &args.add("cell", "Cell size", m_cell);
    m_radiusArg = &args.add("radius", "Minimum radius", m_radius);
}

void SampleFilter::prepared(PointTableRef table)
{
    if (m_cellArg->set() && m_radiusArg->set())
        throwError("Must set only one of 'cell' or 'radius'.");

    if (!m_cellArg->set() && !m_radiusArg->set())
        throwError("Must set 'cell' or 'radius' but not both.");
}

void SampleFilter::ready(PointTableRef)
{
    m_populatedVoxels.clear();

    if (m_cellArg->set())
        m_radius = m_cell / 2.0 * std::sqrt(3.0);

    if (m_radiusArg->set())
        m_cell = 2.0 * m_radius / std::sqrt(3.0);

    log()->get(LogLevel::Debug)
        << "cell " << m_cell << ", radius " << m_radius << std::endl;

    m_radiusSqr = m_radius * m_radius;
}

PointViewSet SampleFilter::run(PointViewPtr view)
{
    PointViewPtr output = view->makeNew();
    for (PointRef point : *view)
    {
        if (voxelize(point))
            output->appendPoint(*view, point.pointId());
    }

    PointViewSet viewSet;
    viewSet.insert(output);
    return viewSet;
}

bool SampleFilter::voxelize(PointRef& point)
{
    double x = point.getFieldAs<double>(Id::X);
    double y = point.getFieldAs<double>(Id::Y);
    double z = point.getFieldAs<double>(Id::Z);

    // Use the coordinates of the first point as origin to offset data and
    // derive integer voxel indices.
    if (m_populatedVoxels.empty())
    {
	m_originX = x;
        m_originY = y;
        m_originZ = z;
    }

    // Get voxel indices for current point.
    Voxel v = std::make_tuple((int)(std::floor((x - m_originX) / m_cell)),
                              (int)(std::floor((y - m_originY) / m_cell)),
                              (int)(std::floor((z - m_originZ) / m_cell)));

    // Check current voxel before any of the neighbors. We will most often have
    // points that are too close in the point's enclosing voxel, thus saving
    // cycles.
    if (m_populatedVoxels.find(v) != m_populatedVoxels.end())
    {
        // Get list of coordinates in candidate voxel.
        CoordList coords = m_populatedVoxels[v];
        for (Coord const& coord : coords)
        {
            // Compute Euclidean distance between current point and
            // candidate voxel.
            double xv = std::get<0>(coord);
            double yv = std::get<1>(coord);
            double zv = std::get<2>(coord);
            double distSqr =
                (xv - x) * (xv - x) + (yv - y) * (yv - y) + (zv - z) * (zv - z);

            // If any point is closer than the minimum radius, we can
            // immediately return false, as the minimum distance
            // criterion is violated.
            if (distSqr < m_radiusSqr)
                return false;
        }
    }

    // Iterate over immediate neighbors of current voxel, computing minimum
    // distance between any already added point and the current point.
    for (int xi = std::get<0>(v) - 1; xi < std::get<0>(v) + 2; ++xi)
    {
        for (int yi = std::get<1>(v) - 1; yi < std::get<1>(v) + 2; ++yi)
        {
            for (int zi = std::get<2>(v) - 1; zi < std::get<2>(v) + 2; ++zi)
            {
                Voxel candidate = std::make_tuple(xi, yi, zi);

                // We have already visited the center voxel, and can skip it.
                if (v == candidate)
                    continue;

                // Check that candidate voxel is occupied.
                if (m_populatedVoxels.find(candidate) ==
                    m_populatedVoxels.end())
                    continue;

                // Get list of coordinates in candidate voxel.
                CoordList coords = m_populatedVoxels[candidate];
                for (Coord const& coord : coords)
                {
                    // Compute Euclidean distance between current point and
                    // candidate voxel.
                    double xv = std::get<0>(coord);
                    double yv = std::get<1>(coord);
                    double zv = std::get<2>(coord);
                    double distSqr = (xv - x) * (xv - x) + (yv - y) * (yv - y) +
                                     (zv - z) * (zv - z);

                    // If any point is closer than the minimum radius, we can
                    // immediately return false, as the minimum distance
                    // criterion is violated.
                    if (distSqr < m_radiusSqr)
                        return false;
                }
            }
        }
    }

    Coord coord = std::make_tuple(x, y, z);
    if (m_populatedVoxels.find(v) != m_populatedVoxels.end())
    {
        m_populatedVoxels[v].push_back(coord);
    }
    else
    {
        CoordList coords;
        coords.push_back(coord);
        m_populatedVoxels.emplace(v, coords);
    }
    return true;
}

bool SampleFilter::processOne(PointRef& point)
{
    return voxelize(point);
}

} // namespace pdal
