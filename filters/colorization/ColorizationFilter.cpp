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
#include <pdal/pdal_macros.hpp>

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
    GlobalEnvironment::get().wakeGDALDrivers();
}


Options ColorizationFilter::getDefaultOptions()
{
    Options options;

    options.add("dimensions", "Red:1:1.0, Green:2:1.0, Blue:3");

    return options;
}

namespace
{

ColorizationFilter::BandInfo parseDim(const std::string& dim,
    uint32_t defaultBand)
{
    std::string::size_type pos, count;
    const char *start;
    char *end;
    std::string name;
    uint32_t band = defaultBand;
    double scale = 1.0;

    try
    {
        pos = 0;
        // Skip leading whitespace.
        count = Utils::extract(dim, pos, (int(*)(int))std::isspace);
        pos += count;

        count = Dimension::extractName(dim, pos);
        if (count == 0)
           throw std::string("No dimension name provided.");
        name = dim.substr(pos, count);
        pos += count;

        count = Utils::extract(dim, pos, (int(*)(int))std::isspace);
        pos += count;

        if (pos < dim.size() && dim[pos] == ':')
        {
            pos++;
            start = dim.data() + pos;
            band = std::strtoul(start, &end, 10);
            if (start == end)
                band = defaultBand;
            pos += (end - start);

            count = Utils::extract(dim, pos, (int(*)(int))std::isspace);
            pos += count;

            if (pos < dim.size() && dim[pos] == ':')
            {
                pos++;
                start = dim.data() + pos;
                scale = std::strtod(start, &end);
                if (start == end)
                    scale = 1.0;
                pos += (end - start);
            }
        }

        count = Utils::extract(dim, pos, (int(*)(int))std::isspace);
        pos += count;

        if (pos != dim.size())
        {
            std::ostringstream oss;

            oss << "Invalid character '" << dim[pos] <<
                "' following dimension specification.";
            throw pdal_error(oss.str());
        }
    }
    catch (std::string s)
    {
        std::ostringstream oss;
        oss << "filters.colorization: invalid --dimensions option: '" << dim <<
            "': " << s;
        throw pdal_error(oss.str());
    }
    return ColorizationFilter::BandInfo(name, band, scale);
}

} // unnamed namespace

void ColorizationFilter::processOptions(const Options& options)
{
    m_rasterFilename = options.getValueOrThrow<std::string>("raster");

    if (options.hasOption("dimension") && !options.hasOption("dimensions"))
        throw pdal_error("Option 'dimension' no longer supported.  Use "
            "'dimensions' instead.");

    StringList defaultDims;
    defaultDims.push_back("Red");
    defaultDims.push_back("Green");
    defaultDims.push_back("Blue");

    StringList dims =
        options.getValueOrDefault<StringList>("dimensions", defaultDims);

    uint32_t defaultBand = 1;
    for (std::string& dim : dims)
    {
        BandInfo bi = parseDim(dim, defaultBand);
        defaultBand = bi.m_band + 1;
        m_bands.push_back(bi);
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
    using namespace gdal;

    m_raster.reset(new gdal::Raster(m_rasterFilename));

    GDALError::Enum error = m_raster->open();
    if (error != GDALError::None)
    {
        if (error == GDALError::NoTransform ||
            error == GDALError::NotInvertible)
        {
            log()->get(LogLevel::Warning) << getName() << ": " <<
                m_raster->errorMsg() << std::endl;
        }
        else
        {
            throw pdal_error(getName() + ": " + m_raster->errorMsg());
        }
    }
}


bool ColorizationFilter::processOne(PointRef& point)
{
    static std::vector<double> data;

    double x = point.getFieldAs<double>(Dimension::Id::X);
    double y = point.getFieldAs<double>(Dimension::Id::Y);

    if (m_raster->read(x, y, data) == gdal::GDALError::None)
    {
        int i(0);
        for (auto bi = m_bands.begin(); bi != m_bands.end(); ++bi)
        {
            BandInfo& b = *bi;
            point.setField(b.m_dim, data[i] * b.m_scale);
            ++i;
        }
        return true;
    }
    return false;
}

void ColorizationFilter::filter(PointView& view)
{
    PointRef point = view.point(0);
    for (PointId idx = 0; idx < view.size(); ++idx)
    {
        point.setPointId(idx);
        processOne(point);
    }
}

} // namespace pdal
