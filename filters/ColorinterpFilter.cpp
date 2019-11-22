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

#include <pdal/GDALUtils.hpp>
#include <pdal/PointView.hpp>
#include <pdal/util/ProgramArgs.hpp>

#include <array>
#include <algorithm>
#include <cmath>

#include "ColorInterpRamps.hpp"

namespace pdal
{

static std::vector<std::string> ramps = {"awesome_green", "black_orange",
    "blue_hue", "blue_red", "heat_map", "pestel_shades", "blue_orange"};

static StaticPluginInfo const s_info
{
    "filters.colorinterp",
    "Assigns RGB colors based on a dimension and a ramp",
    "http://pdal.io/stages/filters.colorinterp.html"
};

CREATE_STATIC_STAGE(ColorinterpFilter, s_info)

std::string ColorinterpFilter::getName() const { return s_info.name; }

// The VSIFILE* that VSIFileFromMemBuffer creates in this
// macro is never cleaned up. We're opening seven PNGs in the
// ColorInterpRamps-ramps.hpp header. We always open them so they're available.
//
// GDAL forces to keep track of the return value, and its being ignored here,
// To avoid the warning message:
// warning: ignoring return value of 'VSILFILE* VSIFileFromMemBuffer(....)'
//          declared with attribute warn_unused_result [-Wunused-result]
// Using a tmp variable
#define GETRAMP(name) \
    if (pdal::Utils::iequals(#name, rampFilename)) \
    { \
        unsigned char* location(name); \
        int size (0); \
        location = name; \
        size = sizeof(name); \
        rampFilename = "/vsimem/" + std::string(#name) + ".png"; \
        auto tmp(VSIFileFromMemBuffer(rampFilename.c_str(), location, size, FALSE)); \
    }
//
std::shared_ptr<gdal::Raster> openRamp(std::string& rampFilename)
{
    // If the user selected a default ramp name, it will be opened by
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
    GETRAMP(blue_orange);

    std::shared_ptr<gdal::Raster>
        output(new gdal::Raster(rampFilename.c_str()));
    return output;
}

void ColorinterpFilter::addArgs(ProgramArgs& args)
{
    args.add("dimension", "Dimension to interpolate", m_interpDimString, "Z");
    args.add("minimum", "Minimum value to use for scaling", m_min,
        std::numeric_limits<double>::quiet_NaN());
    args.add("maximum", "Maximum value to use for scaling", m_max,
        std::numeric_limits<double>::quiet_NaN());
    args.add("ramp", "GDAL-readable color ramp image to use", m_colorramp,
        "pestel_shades");
    args.add("invert", "Invert the ramp direction", m_invertRamp, false);
    args.add("mad", "Use Median Absolute Deviation to compute ramp bounds "
        "in combination with 'k' ", m_useMAD, false);
    args.add("mad_multiplier", "MAD threshold multiplier",
        m_madMultiplier, 1.4862);
    args.add("k", "Number of deviations to compute minimum/maximum ",
        m_stdDevThreshold, 0.0);
}

void ColorinterpFilter::addDimensions(PointLayoutPtr layout)
{
    layout->registerDims({Dimension::Id::Red,
        Dimension::Id::Green, Dimension::Id::Blue});
}


void ColorinterpFilter::prepared(PointTableRef table)
{
    PointLayoutPtr layout(table.layout());
    m_interpDim = layout->findDim(m_interpDimString);
    if (m_interpDim == Dimension::Id::Unknown)
        throwError("Dimension '" + m_interpDimString + "' does not exist.");
    if (!std::isnan(m_min) && !std::isnan(m_max) && m_max <= m_min)
        throwError("Specified 'minimum' value must be less than "
            "'maximum' value.");
}


/**
  Read the band data into local vectors.

  \param table  Point table.
*/
void ColorinterpFilter::ready(PointTableRef table)
{
    gdal::registerDrivers();

    m_raster = openRamp(m_colorramp);
    gdal::GDALError err = m_raster->open();
    if (err != gdal::GDALError::None && err != gdal::GDALError::NoTransform)
        throwError(m_raster->errorMsg());

    log()->get(LogLevel::Debug) << getName() << " raster connection: " <<
        m_raster->filename() << std::endl;

    m_raster->readBand(m_redBand, 1);
    m_raster->readBand(m_greenBand, 2);
    m_raster->readBand(m_blueBand, 3);
}


void ColorinterpFilter::filter(PointView& view)
{
    double median(0.0);
    double mad(0.0);

    // If the user set a 'k' value, we use that
    // defaulting to using a computed stddev if we
    // were not told to use MAD.
    if (m_stdDevThreshold != 0.0)
    {
        std::vector<double> values(view.size());

        stats::Summary summary(Dimension::name(m_interpDim),
            stats::Summary::NoEnum, false);
        for (PointId idx = 0; idx < view.size(); ++idx)
        {
            double v = view.getFieldAs<double>(m_interpDim, idx);
            summary.insert(v);
            values[idx] = v;
        }

        auto compute_median = [](std::vector<double> vals)
        {
            std::nth_element(vals.begin(), vals.begin() + vals.size() / 2,
                vals.end());

            return *(vals.begin() + vals.size() / 2);
        };

        median = compute_median(values);
        if (m_useMAD)
        {
             std::transform(values.begin(), values.end(), values.begin(),
                   [median](double v) { return std::fabs(v - median); });
             mad = compute_median(values);

             double threshold = mad * m_madMultiplier * m_stdDevThreshold;
             m_min = median - threshold;
             m_max = median + threshold;

             log()->get(LogLevel::Debug) << getName() << " mad " <<
                mad << std::endl;
             log()->get(LogLevel::Debug) << getName() << " median " <<
                median << std::endl;
             log()->get(LogLevel::Debug) << getName() << " minimum " <<
                m_min << std::endl;
             log()->get(LogLevel::Debug) << getName() << " maximum " <<
                m_max << std::endl;
        }
        else
        {
             double threshold = (m_stdDevThreshold * summary.sampleStddev());
             m_min = median - threshold;
             m_max = median + threshold;

             log()->get(LogLevel::Debug) << getName() <<
                " stddev threshold " << threshold << std::endl;
             log()->get(LogLevel::Debug) << getName() << " median " <<
                median << std::endl;
             log()->get(LogLevel::Debug) << getName() << " minimum " <<
                m_min << std::endl;
             log()->get(LogLevel::Debug) << getName() << " maximum " <<
                m_max << std::endl;
        }
    }

    // If the user didn't set min/max values and hadn't set a stddev, we
    // compute them.
    else if (std::isnan(m_min) || std::isnan(m_max))
    {
        stats::Summary summary(Dimension::name(m_interpDim),
            stats::Summary::NoEnum, false);
        for (PointId idx = 0; idx < view.size(); ++idx)
        {
            double v = view.getFieldAs<double>(m_interpDim, idx);
            summary.insert(v);
        }

        if (std::isnan(m_min))
            m_min = summary.minimum();
        if (std::isnan(m_max))
            m_max = summary.maximum();
    }

    PointRef point(view, 0);
    for (PointId idx = 0; idx < view.size(); ++idx)
    {
        point.setPointId(idx);
        processOne(point);
    }
}


bool ColorinterpFilter::pipelineStreamable() const
{
    if (std::isnan(m_min) || std::isnan(m_max))
        return false;
    return Streamable::pipelineStreamable();
}


bool ColorinterpFilter::processOne(PointRef& point)
{
    double v = point.getFieldAs<double>(m_interpDim);

    // Don't color points that aren't in the min/max range.
    if (v < m_min || v >= m_max)
        return true;

    double factor = (v - m_min) / (m_max - m_min);
    size_t img_width = m_redBand.size();
    size_t position = size_t(std::floor(factor * img_width));

    if (m_invertRamp)
        position = (img_width - 1) - position;

    point.setField(Dimension::Id::Red, m_redBand[position]);
    point.setField(Dimension::Id::Blue, m_blueBand[position]);
    point.setField(Dimension::Id::Green, m_greenBand[position]);

    return true;
}

} // namespace pdal
