/******************************************************************************
* Copyright (c) 2017, Howard Butler (info@hobu.co)
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

#include "DEMFilter.hpp"

#include <string>
#include <vector>

#include <pdal/private/gdal/Raster.hpp>
#include "private/DimRange.hpp"

namespace pdal
{

static StaticPluginInfo const s_info
{
    "filters.dem",
    "Filter points about an elevation surface",
    "http://pdal.io/stages/filters.dem.html"
};

CREATE_STATIC_STAGE(DEMFilter, s_info)


struct DEMArgs
{
    Dimension::Id m_dim;
    DimRange m_range;
    std::string m_raster;
    int32_t m_band;
};


DEMFilter::DEMFilter() : m_args(new DEMArgs)
{}

DEMFilter::~DEMFilter()
{}

std::string DEMFilter::getName() const
{
    return s_info.name;
}

void DEMFilter::addDimensions(PointLayoutPtr layout)
{
}


void DEMFilter::addArgs(ProgramArgs& args)
{
    args.add("limits", "Dimension limits for filtering", m_args->m_range).setPositional();
    args.add("raster", "GDAL-readable raster to use for DEM", m_args->m_raster).setPositional();
    args.add("band", "Band number to filter (count from 1)", m_args->m_band, 1);

}

void DEMFilter::ready(PointTableRef table)
{
    m_raster.reset(new gdal::Raster(m_args->m_raster));
    m_raster->open();
}


void DEMFilter::prepared(PointTableRef table)
{
    const PointLayoutPtr layout(table.layout());
    m_args->m_dim = layout->findDim(m_args->m_range.m_name);
    if (m_args->m_dim == Dimension::Id::Unknown)
        throwError("Missing dimension with name '" + m_args->m_range.m_name +
            "'in input PointView.");
    if (m_args->m_band <= 0)
        throwError("Band must be greater than 0");

}


bool DEMFilter::processOne(PointRef& point)
{
    static std::vector<double> data;

    double x = point.getFieldAs<double>(Dimension::Id::X);
    double y = point.getFieldAs<double>(Dimension::Id::Y);
    double z = point.getFieldAs<double>(m_args->m_dim);

    bool passes(false);

    if (m_raster->read(x, y, data) == gdal::GDALError::None)
    {
        double v = data[m_args->m_band-1];
        double lb = v - m_args->m_range.m_lower_bound;
        double ub = v + m_args->m_range.m_upper_bound;


        if ( z >= lb && z <= ub)
            passes = true;
    }
    return passes;
}

PointViewSet DEMFilter::run(PointViewPtr inView)
{
    PointViewSet viewSet;
    if (!inView->size())
        return viewSet;

    PointViewPtr outView = inView->makeNew();

    for (PointId i = 0; i < inView->size(); ++i)
    {
        PointRef point = inView->point(i);
        if (processOne(point))
            outView->appendPoint(*inView, i);
    }

    viewSet.insert(outView);
    return viewSet;
}


} // namespace pdal
