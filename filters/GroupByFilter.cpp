/******************************************************************************
 * Copyright (c) 2016-2017, Bradley J Chambers (brad.chambers@gmail.com)
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

#include "GroupByFilter.hpp"

#include <pdal/util/ProgramArgs.hpp>

#include "private/DimRange.hpp"

namespace pdal
{

static PluginInfo const s_info = PluginInfo(
    "filters.groupby", "Group data categorically by dimension or range.",
    "http://pdal.io/stages/filters.groupby.html");

CREATE_STATIC_PLUGIN(1, 0, GroupByFilter, Filter, s_info)

GroupByFilter::GroupByFilter() : m_viewMap()
{
}

std::string GroupByFilter::getName() const
{
    return s_info.name;
}

void GroupByFilter::addArgs(ProgramArgs& args)
{
    m_dimArg = &args.add("dimension", "Dimension containing data to be grouped",
                         m_dimName);
    m_rngArg = &args.add("ranges", "Ranges to be grouped", m_rangeSpec);
}

void GroupByFilter::initialize()
{
    if (m_dimArg->set() && m_rngArg->set())
        throwError(
            "Can't specify both option 'dimension' and option 'ranges'.");
    if (!m_dimArg->set() && !m_rngArg->set())
        throwError(
            "Must specify either option 'dimension' or option 'ranges'.");

    if (m_rngArg->set())
    {
        // Would be better to have the range know how to read from an input
        // stream.
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
                throwError("Invalid 'ranges' option: '" + r +
                           "': " + err.what());
            }
        }
    }
}

void GroupByFilter::prepared(PointTableRef table)
{
    const PointLayoutPtr layout(table.layout());

    if (m_dimArg->set())
    {
        m_dimId = layout->findDim(m_dimName);
        if (m_dimId == Dimension::Id::Unknown)
            throwError("Invalid dimension name '" + m_dimName + "'.");
        // also need to check that we have a dimension with discrete values
    }

    if (m_rngArg->set())
    {
        for (auto& r : m_range_list)
        {
            r.m_id = layout->findDim(r.m_name);
            if (r.m_id == Dimension::Id::Unknown)
                throwError("Invalid dimension name in 'ranges' option: '" +
                           r.m_name + "'.");
        }
        std::sort(m_range_list.begin(), m_range_list.end());
    }
}

PointViewSet GroupByFilter::run(PointViewPtr inView)
{
    PointViewSet viewSet;
    if (!inView->size())
        return viewSet;

    if (m_dimArg->set())
    {
        for (PointId idx = 0; idx < inView->size(); idx++)
        {
            uint64_t val = inView->getFieldAs<uint64_t>(m_dimId, idx);
            PointViewPtr& outView = m_viewMap[val];
            if (!outView)
                outView = inView->makeNew();
            outView->appendPoint(*inView.get(), idx);
        }

        // Pull the buffers out of the map and stick them in the standard
        // output set.
        for (auto bi = m_viewMap.begin(); bi != m_viewMap.end(); ++bi)
            viewSet.insert(bi->second);
    }
    else if (m_rngArg->set())
    {
        for (auto const& r : m_range_list)
        {
            PointViewPtr outView = inView->makeNew();
            for (PointId idx = 0; idx < inView->size(); idx++)
            {
                PointRef point = inView->point(idx);
                if (r.valuePasses(point.getFieldAs<double>(r.m_id)))
                    outView->appendPoint(*inView, idx);
            }
            viewSet.insert(outView);
        }
    }

    return viewSet;
}

} // pdal
