/******************************************************************************
 * Copyright (c) 2016, Bradley J Chambers (brad.chambers@gmail.com)
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

#include "LocateMaxFilter.hpp"

#include <pdal/pdal_macros.hpp>
#include <pdal/util/ProgramArgs.hpp>

namespace pdal
{

static PluginInfo const s_info =
    PluginInfo("filters.locatemax",
               "Return a single point with max value in the named dimension.",
               "http://pdal.io/stages/filters.locatemax.html");

CREATE_STATIC_PLUGIN(1, 0, LocateMaxFilter, Filter, s_info)

std::string LocateMaxFilter::getName() const
{
    return s_info.name;
}

void LocateMaxFilter::addArgs(ProgramArgs& args)
{
    args.add("dimension", "Dimension in which to locate max", m_dimName);
}

void LocateMaxFilter::prepared(PointTableRef table)
{
    PointLayoutPtr layout(table.layout());
    m_dimId = layout->findDim(m_dimName);
    if (m_dimId == Dimension::Id::Unknown)
    {
        std::ostringstream oss;
        oss << "Invalid dimension name in filters.locatemax 'dimension' "
            "option: '" << m_dimName << "'.";
        throw pdal_error(oss.str());
    }
}

PointViewSet LocateMaxFilter::run(PointViewPtr inView)
{
    PointViewSet viewSet;
    if (!inView->size())
        return viewSet;

    PointId maxidx;
    double maxval = std::numeric_limits<double>::lowest();

    for (PointId idx = 0; idx < inView->size(); idx++)
    {
        double val = inView->getFieldAs<double>(m_dimId, idx);
        if (val > maxval)
        {
            maxval = val;
            maxidx = idx;
        }
    }

    PointViewPtr outView = inView->makeNew();
    outView->appendPoint(*inView.get(), maxidx);

    viewSet.insert(outView);
    return viewSet;
}

} // pdal
