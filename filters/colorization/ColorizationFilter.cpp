/******************************************************************************
* Copyright (c) 2012, Howard Butler, hobu.inc@gmail.com
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

#include "ColorizationFilter.hpp"

#include <pdal/GlobalEnvironment.hpp>
#include <pdal/PointView.hpp>

#include <gdal.h>
#include <ogr_spatialref.h>

#include <array>

namespace pdal
{

static PluginInfo const s_info = PluginInfo(
    "filters.colorization",
    "Fetch and assign RGB color information from a GDAL-readable datasource.",
    "http://pdal.io/stages/filters.colorization.html" );

CREATE_STATIC_PLUGIN(1, 0, ColorizationFilter, Filter, s_info)

std::string ColorizationFilter::getName() const { return s_info.name; }


void ColorizationFilter::initialize()
{
    GlobalEnvironment::get().initializeGDAL(log());
}


Options ColorizationFilter::getDefaultOptions()
{
    Options options;

    pdal::Option red("dimension", "Red", "");
    pdal::Option b0("band",1, "");
    pdal::Option s0("scale", 1.0f, "scale factor for this dimension");
    pdal::Options redO;
    redO.add(b0);
    redO.add(s0);
    red.setOptions(redO);

    pdal::Option green("dimension", "Green", "");
    pdal::Option b1("band",2, "");
    pdal::Option s1("scale", 1.0f, "scale factor for this dimension");
    pdal::Options greenO;
    greenO.add(b1);
    greenO.add(s1);
    green.setOptions(greenO);

    pdal::Option blue("dimension", "Blue", "");
    pdal::Option b2("band",3, "");
    pdal::Option s2("scale", 1.0f, "scale factor for this dimension");
    pdal::Options blueO;
    blueO.add(b2);
    blueO.add(s2);
    blue.setOptions(blueO);

    pdal::Option reproject("reproject", false,
        "Reproject the input data into the same coordinate system as "
        "the raster?");

    options.add(red);
    options.add(green);
    options.add(blue);
    options.add(reproject);

    return options;
}


void ColorizationFilter::processOptions(const Options& options)
{
    m_rasterFilename = options.getValueOrThrow<std::string>("raster");
    std::vector<Option> dimensions = options.getOptions("dimension");

    if (dimensions.size() == 0)
    {
        m_bands.emplace_back("Red", Dimension::Id::Red, 1, 1.0);
        m_bands.emplace_back("Green", Dimension::Id::Green, 2, 1.0);
        m_bands.emplace_back("Blue", Dimension::Id::Blue, 3, 1.0);
        log()->get(LogLevel::Debug) << "No dimension mappings were given. "
            "Using default mappings." << std::endl;
    }
    for (auto i = dimensions.begin(); i != dimensions.end(); ++i)
    {
        std::string name = i->getValue<std::string>();
        boost::optional<Options const&> dimensionOptions = i->getOptions();
        if (!dimensionOptions)
        {
            std::ostringstream oss;
            oss << "No band and scaling information given for dimension '" <<
                name << "'";
            throw pdal_error(oss.str());
        }
        uint32_t bandId =
            dimensionOptions->getValueOrThrow<uint32_t>("band");
        double scale =
            dimensionOptions->getValueOrDefault<double>("scale", 1.0);
        m_bands.emplace_back(name, Dimension::Id::Unknown, bandId, scale);
    }
}


void ColorizationFilter::addDimensions(PointLayoutPtr layout)
{
    for (auto& band : m_bands)
        band.m_dim = layout->registerOrAssignDim(band.m_name,
            Dimension::defaultType(Dimension::Id::Red));
}


void ColorizationFilter::ready(PointTableRef table)
{
    m_raster = std::unique_ptr<gdal::Raster>(new gdal::Raster(m_rasterFilename));
    m_raster->open();

    for (auto bi = m_bands.begin(); bi != m_bands.end(); ++bi)
    {
        if (bi->m_dim == Dimension::Id::Unknown)
            bi->m_dim = table.layout()->findDim(bi->m_name);
        if (bi->m_dim == Dimension::Id::Unknown)
            throw pdal_error((std::string)"Can't colorize - no dimension " +
                bi->m_name);
    }
}


void ColorizationFilter::filter(PointView& view)
{
    std::vector<double> data;
    int i(0);
    for (PointId idx = 0; idx < view.size(); ++idx)
    {
        int i(0);
        double x = view.getFieldAs<double>(Dimension::Id::X, idx);
        double y = view.getFieldAs<double>(Dimension::Id::Y, idx);

        bool bRead = m_raster->read(x, y, data);

        if (!bRead) continue;

        for (auto bi = m_bands.begin(); bi != m_bands.end(); ++bi)
        {
            gdal::BandInfo& b = *bi;
            view.setField(b.m_dim, idx, data[i] * b.m_scale);
            ++i;
        }
    }
}

} // namespace pdal
