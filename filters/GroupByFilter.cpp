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

#include "GroupByFilter.hpp"

#include <pdal/util/ProgramArgs.hpp>

namespace pdal
{

static StaticPluginInfo const s_info{
    "filters.groupby", "Split data categorically by dimension.",
    "https://pdal.org/stages/filters.groupby.html"};
CREATE_STATIC_STAGE(GroupByFilter, s_info)

GroupByFilter::GroupByFilter() : m_viewMap() {}

std::string GroupByFilter::getName() const
{
    return s_info.name;
}

void GroupByFilter::addArgs(ProgramArgs& args)
{
    args.add("dimension", "Dimension containing data to be grouped",
             m_dimNames);
}

void GroupByFilter::prepared(PointTableRef table)
{
    StringList badNames;
    PointLayoutPtr layout(table.layout());

    for (const auto& name : m_dimNames)
    {
        auto id = layout->findDim(name);
        if (id == Dimension::Id::Unknown)
            badNames.push_back(name);
        else
            m_dimIds.push_back(id);

        // TODO also check that dimensions are discrete valued (ints?)
    }
    if (!badNames.empty())
    {
        std::stringstream showBad;
        for (const auto& name : badNames)
            showBad << name << " ";
        throwError("Invalid dimension name(s): " + showBad.str() + ".");
    }
}

PointViewSet GroupByFilter::run(PointViewPtr inView)
{
    PointViewSet viewSet;

    for (PointId idx = 0; idx < inView->size(); idx++)
    {
        std::vector<int64_t> key(m_dimIds.size()); // could hash int values
        for (const auto& dimId : m_dimIds)
        {
            int64_t val = inView->getFieldAs<int64_t>(dimId, idx);
            key.push_back(val);
        }

        PointViewPtr& outView = m_viewMap[key];
        if (!outView)
            outView = inView->makeNew();
        outView->appendPoint(*inView.get(), idx);
    }

    // Pull the buffers out of the map and stick them in the standard
    // output set.
    for (auto bi = m_viewMap.begin(); bi != m_viewMap.end(); ++bi)
        viewSet.insert(bi->second);
    return viewSet;
}

} // namespace pdal
