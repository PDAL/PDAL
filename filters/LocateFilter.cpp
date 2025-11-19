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

#include "LocateFilter.hpp"

#include <pdal/util/ProgramArgs.hpp>
#include <pdal/util/Utils.hpp>

namespace pdal
{

static StaticPluginInfo const s_info
{
    "filters.locate",
    "Return a single point with min/max value in the named dimension.",
    "http://pdal.org/stages/filters.locate.html"
};

CREATE_STATIC_STAGE(LocateFilter, s_info)

std::string LocateFilter::getName() const
{
    return s_info.name;
}

void LocateFilter::addArgs(ProgramArgs& args)
{
    args.add("dimension", "Dimension in which to locate max", m_dimName);
    args.add("minmax", "Whether to search for the minimum or maximum value",
        m_minmax, "max");
}

void LocateFilter::prepared(PointTableRef table)
{
    PointLayoutPtr layout(table.layout());
    m_dimId = layout->findDim(m_dimName);
    if (m_dimId == Dimension::Id::Unknown)
        throwError("Invalid dimension '" + m_dimName + "'.");
}

PointViewSet LocateFilter::run(PointViewPtr inView)
{

    PointViewSet viewSet;
    PointId minidx(0);
    PointId maxidx(0);
    double minval = (std::numeric_limits<double>::max)();
    double maxval = std::numeric_limits<double>::lowest();

    for (PointId idx = 0; idx < inView->size(); idx++)
    {
        double val = inView->getFieldAs<double>(m_dimId, idx);
        if (val > maxval)
        {
            maxval = val;
            maxidx = idx;
        }
        if (val < minval)
        {
            minval = val;
            minidx = idx;
        }
    }

    PointViewPtr outView = inView->makeNew();

    if (Utils::iequals("min", m_minmax))
        outView->appendPoint(*inView.get(), minidx);
    if (Utils::iequals("max", m_minmax))
        outView->appendPoint(*inView.get(), maxidx);

    viewSet.insert(outView);
    return viewSet;
}

} // pdal
