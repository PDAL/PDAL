/******************************************************************************
* Copyright (c) 2020, Julian Fell (hi@jtfell.com)
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

#include "HagDemFilter.hpp"

#include <pdal/private/gdal/Raster.hpp>

namespace pdal
{

static StaticPluginInfo const s_info
{
    "filters.hag_dem",
    "Computes height above ground using a DEM raster.",
    "http://pdal.io/stages/filters.hag_dem.html"
};

CREATE_STATIC_STAGE(HagDemFilter, s_info)

std::string HagDemFilter::getName() const
{
    return s_info.name;
}


HagDemFilter::HagDemFilter()
{}


void HagDemFilter::addArgs(ProgramArgs& args)
{
    args.add("raster", "GDAL-readable raster to use for DEM (uses band 1, "
        "starting from 1)", m_rasterName).setPositional();
    args.add("band", "Band number to filter (count from 1)", m_band, 1);
    args.add("zero_ground", "If true, set HAG of ground-classified points "
        "to 0 rather than comparing Z value to raster DEM",
        m_zeroGround, true);
}


void HagDemFilter::addDimensions(PointLayoutPtr layout)
{
    layout->registerDim(Dimension::Id::HeightAboveGround);
}


void HagDemFilter::ready(PointTableRef table)
{
    m_raster.reset(new gdal::Raster(m_rasterName));
    m_raster->open();
}

void HagDemFilter::prepared(PointTableRef table)
{
    if (m_band <= 0)
        throwError("Band must be greater than 0");
}

void HagDemFilter::filter(PointView& view)
{
    PointRef point(view, 0);
    for (PointId i = 0; i < view.size(); ++i)
    {
        point.setPointId(i);
        processOne(point);
    }
}

bool HagDemFilter::processOne(PointRef& point)
{
    using namespace pdal::Dimension;
    std::vector<double> data;

    // If "zero_ground" option is set, all ground points get HAG of 0
    if (m_zeroGround &&
        point.getFieldAs<uint8_t>(Id::Classification) == ClassLabel::Ground)
    {
        point.setField(Id::HeightAboveGround, 0);
    }
    else
    {
        double x = point.getFieldAs<double>(Id::X);
        double y = point.getFieldAs<double>(Id::Y);

        // If raster has a point at X, Y of pointcloud point, use it.
        // Otherwise the HAG value is not set.
        if (m_raster->read(x, y, data) == gdal::GDALError::None)
        {
            double z = point.getFieldAs<double>(Id::Z);
            double hag = z - data[m_band - 1];
            point.setField(Dimension::Id::HeightAboveGround, hag);
        }
    }
    return true;
}

} // namespace pdal
