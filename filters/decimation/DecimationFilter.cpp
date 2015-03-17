/******************************************************************************
* Copyright (c) 2011, Michael P. Gerlek (mpg@flaxen.com)
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

#include "DecimationFilter.hpp"

#include <pdal/PointView.hpp>

namespace pdal
{

static PluginInfo const s_info = PluginInfo(
    "filters.decimation",
    "Rank decimation filter. Keep every Nth point",
    "http://pdal.io/stages/filters.decimation.html" );

CREATE_STATIC_PLUGIN(1, 0, DecimationFilter, Filter,  s_info)

std::string DecimationFilter::getName() const { return s_info.name; }

void DecimationFilter::processOptions(const Options& options)
{
    m_step = options.getValueOrDefault<uint32_t>("step", 1);
    m_offset = options.getValueOrDefault<uint32_t>("offset", 0);
    m_limit = options.getValueOrDefault<point_count_t>("limit", 0);
}


PointViewSet DecimationFilter::run(PointViewPtr inView)
{
    PointViewSet viewSet;
    PointViewPtr outView = inView->makeNew();
    decimate(*inView.get(), *outView.get());
    viewSet.insert(outView);
    return viewSet;
}


void DecimationFilter::decimate(PointView& input, PointView& output)
{
    PointId last_idx = (m_limit > 0) ? m_limit : input.size();
    for (PointId idx = m_offset; idx < last_idx; idx += m_step)
        output.appendPoint(input, idx);
}

} // pdal
