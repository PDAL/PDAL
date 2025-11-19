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
#include <pdal/util/ProgramArgs.hpp>

namespace pdal
{

static StaticPluginInfo const s_info
{
    "filters.decimation",
    "Rank decimation filter. Keep every Nth point",
    "http://pdal.org/stages/filters.decimation.html"
};

CREATE_STATIC_STAGE(DecimationFilter, s_info)

std::string DecimationFilter::getName() const { return s_info.name; }

void DecimationFilter::addArgs(ProgramArgs& args)
{
    args.add("step", "Points to delete between each kept point", m_step, 1.0);
    args.add("offset", "Index of first point to consider including in output",
        m_offset);
    args.add("limit", "Index of last point to consider including in output",
        m_limit, (std::numeric_limits<point_count_t>::max)());
}

void DecimationFilter::initialize()
{
    if (m_step < 1.0)
        throwError("Option step must be >= 1.0");
}

PointViewSet DecimationFilter::run(PointViewPtr inView)
{
    PointViewSet viewSet;
    PointViewPtr outView = inView->makeNew();
    decimate(*inView.get(), *outView.get());
    viewSet.insert(outView);
    return viewSet;
}


bool DecimationFilter::processOne(PointRef& point)
{
    bool keep = true;
    if (m_index < m_offset || m_index >= m_limit)
        keep = false;
    else if (m_index != (m_offset + std::llround(m_kept * m_step)))
        keep = false;
    else
        m_kept++;

    m_index++;
    return keep;
}


void DecimationFilter::decimate(PointView& input, PointView& output)
{

    uint64_t last_idx = (std::min)(m_limit, input.size());
    uint64_t count = std::llround((last_idx - m_offset) / m_step);

    for (uint64_t idx = 0; idx < count; ++idx)
        output.appendPoint(input, m_offset + std::llround(idx * m_step));
}

} // pdal
