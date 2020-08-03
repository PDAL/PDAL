/******************************************************************************
 * Copyright (c) 2017, Bradley J Chambers (brad.chambers@gmail.com)
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

#include "HeadFilter.hpp"

namespace pdal
{

static StaticPluginInfo const s_info
{
    "filters.head",
    "Return N points from beginning of the point cloud.",
    "http://pdal.io/stages/filters.head.html"
};

CREATE_STATIC_STAGE(HeadFilter, s_info)

std::string HeadFilter::getName() const
{
    return s_info.name;
}

void HeadFilter::addArgs(ProgramArgs& args)
{
    args.add("count", "Number of points to return from beginning.  "
        "If 'invert' is true, number of points to drop from the beginning.",
        m_count, point_count_t(10));
    args.add("invert", "If true, 'count' specifies the number of points "
        "to skip from the beginning.", m_invert, false);
}

void HeadFilter::ready(PointTableRef table)
{
    m_index = 0;
}

bool HeadFilter::processOne(PointRef& point)
{

    bool keep = false;
    if (m_index < m_count)
        keep = true;
    m_index++;

    if (m_invert)
        keep = !keep;
    return keep;


}

PointViewSet HeadFilter::run(PointViewPtr inView)
{
    PointViewSet viewSet;
    if (!inView->size())
        return viewSet;

    PointViewPtr outView = inView->makeNew();

    for (PointId i = 0; i < inView->size(); ++i)
    {
        PointRef point = inView->point(i);
        if (processOne(point))
            outView->appendPoint(*inView, i);
    }

    viewSet.insert(outView);
    return viewSet;
}

} // namespace pdal
