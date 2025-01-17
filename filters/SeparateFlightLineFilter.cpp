/******************************************************************************
 * Copyright (c) 2025, Johnathan Tenny (jt893@nau.edu)
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

#include "SeparateFlightLineFilter.hpp"

#include <pdal/util/ProgramArgs.hpp>

namespace pdal
{

static StaticPluginInfo const s_info
{
    "filters.separateflightline",
    "Split data by flight line based on a gap in GpsTime.",
    "http://pdal.io/stages/filters.separateflightline.html"
};
CREATE_STATIC_STAGE(SeparateFlightLineFilter, s_info)

SeparateFlightLineFilter::SeparateFlightLineFilter()
{}

std::string SeparateFlightLineFilter::getName() const
{
    return s_info.name;
}

void SeparateFlightLineFilter::addArgs(ProgramArgs& args)
{
    args.add("time_gap", "Minimum amount of GpsTime that separates two flight lines", m_timeGap, 5.0f);
}

void SeparateFlightLineFilter::prepared(PointTableRef table)
{
    PointLayoutPtr layout(table.layout());
    if (!layout->hasDim(Dimension::Id::GpsTime))
        throwError("Layout does not contain GpsTime dimension.");
}

PointViewSet SeparateFlightLineFilter::run(PointViewPtr inView)
{
    PointViewSet result;
    PointViewPtr v(inView->makeNew());
    result.insert(v);

    float gpsTime = 0;
    float gpsTimeNext = 0;
    for (PointId i = 0; i < inView->size() - 1;++i)
    {
        v->appendPoint(*inView, i);
        gpsTime = inView->getFieldAs<double>(Dimension::Id::GpsTime, i);
        gpsTimeNext = inView->getFieldAs<double>(Dimension::Id::GpsTime, i+1);
        if (gpsTimeNext<gpsTime)
        throwError("View must be sorted by ascending GpsTime")
        if (gpsTimeNext - gpsTime >= m_timeGap)
        {
            v = inView->makeNew();
            result.insert(v);
        }
    }
    //add last point to current view
    if (inView->size() > 0)
    v->appendPoint(*inView, inView->size() - 1);
    
    return result;
}

} // pdal
