/******************************************************************************
 * Copyright (c) 2014, Hobu Inc., hobu.inc@gmail.com
 * Copyright (c) 2015, Bradley J Chambers, brad.chambers@gmail.com
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
 *     * Neither the name of the Andrew Bell or libLAS nor the names of
 *       its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
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

#include "MergeFilter.hpp"

namespace pdal
{

static StaticPluginInfo const s_info
{
    "filters.merge",
    "Merge data from two different readers into a single stream.",
    "http://pdal.org/stages/filters.merge.html"
};

CREATE_STATIC_STAGE(MergeFilter, s_info)

std::string MergeFilter::getName() const { return s_info.name; }

void MergeFilter::ready(PointTableRef table)
{
    SpatialReference srs = getSpatialReference();

    if (srs.empty())
        srs = table.anySpatialReference();
    m_view.reset(new PointView(table, srs));
}


PointViewSet MergeFilter::run(PointViewPtr in)
{
    PointViewSet viewSet;

    // If the SRS of all the point views aren't the same, print a warning
    // unless we're explicitly overriding the SRS.
    if (getSpatialReference().empty() &&
      (in->spatialReference() != m_view->spatialReference()))
        log()->get(LogLevel::Warning) << getName() << ": merging points "
            "with inconsistent spatial references." << std::endl;
    m_view->append(*in.get());
    viewSet.insert(m_view);
    return viewSet;
}

} // namespace pdal

