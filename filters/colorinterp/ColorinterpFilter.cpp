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

#include "ColorinterpFilter.hpp"

#include <pdal/PointView.hpp>
#include <pdal/pdal_macros.hpp>
#include <pdal/util/ProgramArgs.hpp>

#include <gdal.h>
#include <ogr_spatialref.h>

#include <array>

namespace pdal
{

static PluginInfo const s_info = PluginInfo(
    "filters.colorinterp",
    "Assigns RGB colors based on a dimension and a ramp",
    "http://pdal.io/stages/filters.colorinterp.html" );

CREATE_STATIC_PLUGIN(1, 0, ColorinterpFilter, Filter, s_info)

std::string ColorinterpFilter::getName() const { return s_info.name; }


void ColorinterpFilter::addArgs(ProgramArgs& args)
{
    std::string dimension;
    args.add("dimension", "Dimension to interpolate", dimension, "Z");
    args.add("minimum", "Minimum value to use for scaling", m_min);
    args.add("maximum", "Maximum value to use for scaling", m_max);

    m_interpDim = Dimension::id(dimension);

}

void interpolateColor(double value, double minValue, double maxValue, double& red, double& green, double& blue)
{
    // initialize to white
    red = 1.0;
    green = 1.0;
    blue = 1.0;

    if (value < minValue)
    {
        value = minValue;
    }

    if (value > maxValue)
    {
        value = maxValue;
    }

    double dv = maxValue - minValue;

    if (value < (minValue + (0.25 * dv)))
    {
        red = 0;
        green = 4 * (value - minValue) / dv;
    }
    else if (value < (minValue + (0.5 * dv)))
    {
        red = 0;
        blue = 1 + (4 * (minValue + (0.25 * dv) - value) / dv);
    }
    else if (value < (minValue + (0.75 * dv)))
    {
        red = 4 * (value - minValue - (0.5 * dv)) / dv;
        blue = 0;
    }
    else
    {
        green = 1 + (4 * (minValue + (0.75 * dv) - value) / dv);
        blue = 0;
    }

    return;
}

bool ColorinterpFilter::processOne(PointRef& point)
{

    double v = point.getFieldAs<double>(Dimension::Id::Z);
    double red(1.0), green(1.0), blue(1.0);

    interpolateColor(v, m_min, m_max, red, blue, green);

    point.setField(Dimension::Id::Red, red);
    point.setField(Dimension::Id::Green, green);
    point.setField(Dimension::Id::Blue, blue);

    return false;
}

void ColorinterpFilter::filter(PointView& view)
{
    PointRef point = view.point(0);
    for (PointId idx = 0; idx < view.size(); ++idx)
    {
        point.setPointId(idx);
        processOne(point);
    }
}

} // namespace pdal
