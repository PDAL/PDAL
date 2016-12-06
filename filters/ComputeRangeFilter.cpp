/******************************************************************************
* Copyright (c) 2016, Bradley J Chambers (brad.chambers@gmail.com)
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

#include "ComputeRangeFilter.hpp"

#include <pdal/pdal_macros.hpp>

#include <string>

namespace pdal
{

static PluginInfo const s_info =
    PluginInfo("filters.computerange", "Compute Range Filter",
               "http://pdal.io/stages/filters.computerange.html");

CREATE_STATIC_PLUGIN(1, 0, ComputeRangeFilter, Filter, s_info)

std::string ComputeRangeFilter::getName() const
{
    return s_info.name;
}

void ComputeRangeFilter::addDimensions(PointLayoutPtr layout)
{
    using namespace Dimension;
    m_range = layout->registerOrAssignDim("Range", Type::Double);
}

void ComputeRangeFilter::prepared(PointTableRef table)
{
    using namespace Dimension;

    const PointLayoutPtr layout(table.layout());

    m_frameNumber = layout->findDim("Frame Number");
    if (m_frameNumber == Id::Unknown)
        throw pdal_error("ComputeRangeFilter: missing Frame Number dimension in input PointView");

    m_pixelNumber = layout->findDim("Pixel Number");
    if (m_pixelNumber == Id::Unknown)
        throw pdal_error("ComputeRangeFilter: missing Pixel Number dimension in input PointView");
}

void ComputeRangeFilter::filter(PointView& view)
{
    using namespace Dimension;

    // Sensor coordinates are provided for each frame. The pixel number -5 is
    // used to flag the sensor position.
    log()->get(LogLevel::Debug) << "Stash sensor positions...\n";
    struct sp
    {
        double sx, sy, sz;
    };
    std::map<int, sp> smap;
    for (PointId i = 0; i < view.size(); ++i)
    {
        int f = view.getFieldAs<int>(m_frameNumber, i);
        int p = view.getFieldAs<int>(m_pixelNumber, i);
        if (p == -5)
        {
            smap[f].sx = view.getFieldAs<double>(Id::X, i);
            smap[f].sy = view.getFieldAs<double>(Id::Y, i);
            smap[f].sz = view.getFieldAs<double>(Id::Z, i);
        }
    }

    // For each XYZ coordinate, we look up the sensor position for the
    // corresponding frame and compute the Euclidean distance. This is
    // recorded in the Range dimension.
    log()->get(LogLevel::Debug) << "Compute ranges...\n";
    for (PointId i = 0; i < view.size(); ++i)
    {
        int f = view.getFieldAs<int>(m_frameNumber, i);
        double dx = smap[f].sx - view.getFieldAs<double>(Id::X, i);
        double dy = smap[f].sy - view.getFieldAs<double>(Id::Y, i);
        double dz = smap[f].sz - view.getFieldAs<double>(Id::Z, i);
        double r = std::sqrt(dx * dx + dy * dy + dz * dz);
        view.setField(m_range, i, r);
    }
}

} // namespace pdal
