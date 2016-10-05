/******************************************************************************
* Copyright (c) 2016, Howard Butler, hobu.inc@gmail.com
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

#include "ColorinterpFilter.hpp"

#include <pdal/PointView.hpp>
#include <pdal/GDALUtils.hpp>
#include <pdal/pdal_macros.hpp>
#include <pdal/util/ProgramArgs.hpp>

#include <gdal.h>
#include <cpl_vsi.h>
#include <ogr_spatialref.h>

#include <array>

#include "color-ramps.hpp"

namespace pdal
{

static std::vector<std::string> ramps = {"awesome_green", "black_orange", "blue_hue", "blue_red", "heat_map", "pestel_shades", "blue_orange"};

static PluginInfo const s_info = PluginInfo(
    "filters.colorinterp",
    "Assigns RGB colors based on a dimension and a ramp",
    "http://pdal.io/stages/filters.colorinterp.html" );

CREATE_STATIC_PLUGIN(1, 0, ColorinterpFilter, Filter, s_info)

std::string ColorinterpFilter::getName() const { return s_info.name; }


#define GETRAMP(name) \
    if (pdal::Utils::iequals(#name, rampFilename)) \
    { \
        VSILFILE* vsifile; \
        GByte* location(0); \
        int size (0); \
        location = name; \
        size = sizeof(name); \
        rampFilename = "/vsimem/" + std::string(#name) + ".png"; \
        vsifile = VSIFileFromMemBuffer(rampFilename.c_str(), location, size, FALSE); \
    }
//
std::shared_ptr<pdal::gdal::Raster> openRamp(std::string& rampFilename)
{
    // If the user] selected a default ramp name, it will be opened by
    // one of these macros if it matches. Otherwise, we just open with the
    // GDALOpen'able the user gave us

    // GETRAMP will set rampFilename to the /vsimem filename it
    // used to actually open the file, and from then on it can be treated
    // like any other GDAL datasource.

    GETRAMP(awesome_green);
    GETRAMP(black_orange);
    GETRAMP(blue_hue);
    GETRAMP(blue_red);
    GETRAMP(heat_map);
    GETRAMP(pestel_shades);

    std::shared_ptr<pdal::gdal::Raster> output (new pdal::gdal::Raster(rampFilename.c_str()));
    return output;
}

void ColorinterpFilter::addArgs(ProgramArgs& args)
{
    std::string dimension;
    args.add("dimension", "Dimension to interpolate", dimension, "Z");
    args.add("minimum", "Minimum value to use for scaling", m_min);
    args.add("maximum", "Maximum value to use for scaling", m_max);
    args.add("ramp", "GDAL-readable color ramp image to use", m_colorramp, "pestel_shades");
    args.add("invert", "Invert the ramp direction", m_invertRamp, false);
    args.add("stddev", "Compute minimum/maximum given this many standard deviations", m_stdDevThreshold, 0.0);
    m_interpDim = Dimension::id(dimension);
}

void ColorinterpFilter::addDimensions(PointLayoutPtr layout)
{
    layout->registerOrAssignDim("Red",
        Dimension::defaultType(Dimension::Id::Red));
    layout->registerOrAssignDim("Green",
        Dimension::defaultType(Dimension::Id::Green));
    layout->registerOrAssignDim("Blue",
        Dimension::defaultType(Dimension::Id::Blue));
}

void ColorinterpFilter::initialize()
{
    gdal::registerDrivers();

    m_raster = openRamp(m_colorramp);
    m_raster->open();

    log()->get(LogLevel::Debug) << getName() << " raster connection: "
                                             << m_raster->m_filename << std::endl;
}

bool ColorinterpFilter::processOne(PointRef& point)
{

    double v = point.getFieldAs<double>(m_interpDim);
    double red(1.0), green(1.0), blue(1.0);

    size_t img_width = m_redBand.size();
    double factor = (v - m_min) / (m_max - m_min);

    if (m_invertRamp)
        factor = 1 - factor;

    size_t position(std::floor(factor * img_width));

    // Don't color points whose computed position
    // would fall outside the image
    if (position > img_width)
    {
        return false;
    }

    red = m_redBand[position];
    blue = m_blueBand[position];
    green = m_greenBand[position];

    point.setField(Dimension::Id::Red, red);
    point.setField(Dimension::Id::Green, green);
    point.setField(Dimension::Id::Blue, blue);

    return false;
}

void ColorinterpFilter::filter(PointView& view)
{
    if ( m_stdDevThreshold != 0.0)
    {
        std::vector<double> values(view.size());

        pdal::stats::Summary summary(pdal::Dimension::name(m_interpDim), pdal::stats::Summary::NoEnum);
        for (PointId idx = 0; idx < view.size(); ++idx)
        {
            double v = view.getFieldAs<double>(m_interpDim, idx);
            summary.insert(v);
            values[idx] = v;
        }

        auto median = [](std::vector<double> vals)
        {
            std::nth_element(vals.begin(), vals.begin()+vals.size()/2, vals.end());

            return *(vals.begin()+vals.size()/2);
        };

        double med = median(values);
        double threshold = (m_stdDevThreshold * summary.stddev());
        m_min = med - threshold;
        m_max = med + threshold;

        log()->get(LogLevel::Debug) << getName() << " stddev threshold " << threshold << std::endl;
        log()->get(LogLevel::Debug) << getName() << " median " << med << std::endl;
        log()->get(LogLevel::Debug) << getName() << " minimum " << m_min << std::endl;
        log()->get(LogLevel::Debug) << getName() << " maximum " << m_max << std::endl;

    }
    else if ((m_min == 0.0 && m_max == 0.0) )
    {
        pdal::stats::Summary summary(pdal::Dimension::name(m_interpDim), pdal::stats::Summary::NoEnum);
        for (PointId idx = 0; idx < view.size(); ++idx)
        {
            double v = view.getFieldAs<double>(m_interpDim, idx);
            summary.insert(v);
        }

        m_min = summary.minimum();
        m_max = summary.maximum();
    }

    m_raster->readBand(m_redBand, 1);
    m_raster->readBand(m_greenBand, 2 );
    m_raster->readBand(m_blueBand, 3);

    PointRef point = view.point(0);

    for (PointId idx = 0; idx < view.size(); ++idx)
    {
        point.setPointId(idx);
        processOne(point);
    }
}

} // namespace pdal
