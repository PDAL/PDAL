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
#include <algorithm>
#include <cmath>

#include "ColorInterpRamps.hpp"

namespace pdal
{

static std::vector<std::string> ramps = {"awesome_green", "black_orange", "blue_hue", "blue_red", "heat_map", "pestel_shades", "blue_orange"};

static PluginInfo const s_info = PluginInfo(
    "filters.colorinterp",
    "Assigns RGB colors based on a dimension and a ramp",
    "http://pdal.io/stages/filters.colorinterp.html" );

CREATE_STATIC_PLUGIN(1, 0, ColorinterpFilter, Filter, s_info)

std::string ColorinterpFilter::getName() const { return s_info.name; }

// The VSIFILE* that VSIFileFromMemBuffer creates in this
// macro is never cleaned up. We're opening seven PNGs in the
// ColorInterpRamps-ramps.hpp header. We always open them so they're available.
#define GETRAMP(name) \
    if (pdal::Utils::iequals(#name, rampFilename)) \
    { \
        GByte* location(0); \
        int size (0); \
        location = name; \
        size = sizeof(name); \
        rampFilename = "/vsimem/" + std::string(#name) + ".png"; \
        (void)VSIFileFromMemBuffer(rampFilename.c_str(), location, size, FALSE); \
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
    args.add("dimension", "Dimension to interpolate", m_interpDimString, "Z");
    args.add("minimum", "Minimum value to use for scaling", m_min);
    args.add("maximum", "Maximum value to use for scaling", m_max);
    args.add("ramp", "GDAL-readable color ramp image to use", m_colorramp, "pestel_shades");
    args.add("invert", "Invert the ramp direction", m_invertRamp, false);
    args.add("mad", "Use Median Absolute Deviation to compute ramp bounds in combination with 'k' ", m_useMAD, false);
    args.add("mad_multiplier", "MAD threshold multiplier", m_madMultiplier, 1.4862);
    args.add("k", "Number of deviations to compute minimum/maximum ", m_stdDevThreshold, 0.0);
}

void ColorinterpFilter::addDimensions(PointLayoutPtr layout)
{
    layout->registerDims({Dimension::Id::Red, Dimension::Id::Green, Dimension::Id::Blue});
}

void ColorinterpFilter::initialize()
{
    gdal::registerDrivers();

    m_raster = openRamp(m_colorramp);
    m_raster->open();

    log()->get(LogLevel::Debug) << getName() << " raster connection: "
                                             << m_raster->filename() << std::endl;

    m_interpDim = Dimension::id(m_interpDimString);
    if (m_interpDim == Dimension::Id::Unknown)
        throw pdal_error("Dimension name is not known!");
}

void ColorinterpFilter::filter(PointView& view)
{
    double median(0.0);
    double mad(0.0);

    // If the user set a 'k' value, we use that
    // defaulting to using a computed stddev if we
    // were not told to use MAD.
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

        auto compute_median = [](std::vector<double> vals)
        {
            std::nth_element(vals.begin(), vals.begin()+vals.size()/2, vals.end());

            return *(vals.begin()+vals.size()/2);
        };

        median = compute_median(values);
        if (m_useMAD)
        {
             std::transform(values.begin(), values.end(), values.begin(),
                   [median](double v) { return std::fabs(v - median); });
             mad = compute_median(values);

             mad = mad * m_madMultiplier;
             double threshold = m_stdDevThreshold * mad;
             m_min = median - threshold;
             m_max = median + threshold;

             log()->get(LogLevel::Debug) << getName() << " mad " << mad << std::endl;
             log()->get(LogLevel::Debug) << getName() << " median " << median << std::endl;
             log()->get(LogLevel::Debug) << getName() << " minimum " << m_min << std::endl;
             log()->get(LogLevel::Debug) << getName() << " maximum " << m_max << std::endl;
        }
        else
        {
             double threshold = (m_stdDevThreshold * summary.stddev());
             m_min = median - threshold;
             m_max = median + threshold;

             log()->get(LogLevel::Debug) << getName() << " stddev threshold " << threshold << std::endl;
             log()->get(LogLevel::Debug) << getName() << " median " << median << std::endl;
             log()->get(LogLevel::Debug) << getName() << " minimum " << m_min << std::endl;
             log()->get(LogLevel::Debug) << getName() << " maximum " << m_max << std::endl;
        }

    }

    // If the user didn't set min/max values and hadn't set a stddev, we
    // compute them.
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



    for (PointId idx = 0; idx < view.size(); ++idx)
    {

        double v = view.getFieldAs<double>(m_interpDim, idx);

        size_t img_width = m_redBand.size();
        double factor = (v - m_min) / (m_max - m_min);

        if (m_invertRamp)
            factor = 1 - factor;

        size_t position(std::floor(factor * img_width));

        // Don't color points whose computed position
        // would fall outside the image
        if (position > img_width)
        {
            continue;
        }

        double red = m_redBand[position];
        double blue = m_blueBand[position];
        double green = m_greenBand[position];

        view.setField(Dimension::Id::Red, idx, red);
        view.setField(Dimension::Id::Green, idx, green);
        view.setField(Dimension::Id::Blue, idx, blue);

    }
}

} // namespace pdal
