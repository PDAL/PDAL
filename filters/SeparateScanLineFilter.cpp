/******************************************************************************
 * Copyright (c) 2019, Guilhem Villemin (guilhem.villemin@gmail.com)
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

#include "SeparateScanLineFilter.hpp"

#include <pdal/util/ProgramArgs.hpp>

namespace pdal
{

static StaticPluginInfo const s_info
{
    "filters.separatescanline",
    "Split data by scan line.",
    "https://pdal.org/stages/filters.separatescanline.html"
};
CREATE_STATIC_STAGE(SeparateScanLineFilter, s_info)

SeparateScanLineFilter::SeparateScanLineFilter()
{}

std::string SeparateScanLineFilter::getName() const
{
    return s_info.name;
}

void SeparateScanLineFilter::addArgs(ProgramArgs& args)
{
    args.add("groupby", "Number of lines to be grouped by", m_groupBy, (uint64_t) 1u);
}

void SeparateScanLineFilter::prepared(PointTableRef table)
{
    PointLayoutPtr layout(table.layout());
    if (!layout->hasDim(Dimension::Id::EdgeOfFlightLine))
        throwError("Layout does not contains EdgeOfFlightLine dimension.");
}

PointViewSet SeparateScanLineFilter::run(PointViewPtr inView)
{
    PointViewSet result;
    PointViewPtr v(inView->makeNew());
    result.insert(v);
    
    uint64_t lineNum = 1;
    for (PointId i = 0; i < inView->size();++i)
    {
        v->appendPoint(*inView, i);
        if (inView->getFieldAs<uint8_t>(Dimension::Id::EdgeOfFlightLine, i))
        {
            if (++lineNum > m_groupBy)
            {
                v = inView->makeNew();
                result.insert(v);
                lineNum = 1;
            }
        }
    }
    //if last point was an edge of flight line
    if (v->empty())
        result.erase(v);
    
    return result;
}

} // pdal
