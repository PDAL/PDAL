/******************************************************************************
 * Copyright (c) 2015, Bradley J Chambers (brad.chambers@gmail.com)
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

#include "RangeFilter.hpp"

#include <pdal/pdal_macros.hpp>
#include <pdal/util/ProgramArgs.hpp>
#include <pdal/util/Utils.hpp>

#include "private/DimRange.hpp"

#include <cctype>
#include <limits>
#include <map>
#include <string>
#include <vector>

namespace pdal
{

static PluginInfo const s_info =
    PluginInfo("filters.range", "Pass only points given a dimension/range.",
               "http://pdal.io/stages/filters.range.html");

CREATE_STATIC_PLUGIN(1, 0, RangeFilter, Filter, s_info)

std::string RangeFilter::getName() const
{
    return s_info.name;
}


RangeFilter::RangeFilter()
{}


RangeFilter::~RangeFilter()
{}


void RangeFilter::addArgs(ProgramArgs& args)
{
    args.add("limits", "Range limits", m_rangeSpec).setPositional();
}


void RangeFilter::initialize()
{
    // Would be better to have the range know how to read from an input stream.
    for (auto const& r : m_rangeSpec)
    {
        try
        {
            DimRange range;
            range.parse(r);
            m_range_list.push_back(range);
        }
        catch (const DimRange::error& err)
        {
            throwError("Invalid 'limits' option: '" + r + "': " + err.what());
        }
    }
}


void RangeFilter::prepared(PointTableRef table)
{
    const PointLayoutPtr layout(table.layout());

    for (auto& r : m_range_list)
    {
        r.m_id = layout->findDim(r.m_name);
        if (r.m_id == Dimension::Id::Unknown)
            throwError("Invalid dimension name in 'limits' option: '" +
                r.m_name + "'.");
    }
    std::sort(m_range_list.begin(), m_range_list.end());
}


// The range list is sorted by dimension, so the logic here should work
// as ORs between ranges of the same dimension and ANDs between ranges
// of different dimensions.  This is simple logic, but is probably the most
// common case.
bool RangeFilter::processOne(PointRef& point)
{
    Dimension::Id lastId = m_range_list.front().m_id;
    bool passes = false;
    for (auto const& r : m_range_list)
    {
        // If we're at a new dimension, return false if we haven't passed
        // the dimension, otherwise reset passes to false for the next
        // dimension and keep checking.
        if (r.m_id != lastId)
        {
            if (!passes)
                return false;
            lastId = r.m_id;
            passes = false;
        }
        // If we've already passed this dimension, continue until we find
        // a new dimension.
        else if (passes)
            continue;
        passes = r.valuePasses(point.getFieldAs<double>(r.m_id));
    }
    return passes;
}


PointViewSet RangeFilter::run(PointViewPtr inView)
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
