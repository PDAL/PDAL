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

#include <functional>

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
    args.add("dimension", "1 or more dimensions by which to group data",
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
        throwError("Invalid dimension name(s): " + showBad.str());
    }
}

/**
    Accumulates successive hashes of `value` into a single 64-bit buffer.
    Taken from boost, etc.
    https://www.open-std.org/jtc1/sc22/wg21/docs/papers/2017/p0814r0.pdf

    \param hash  The accumulated 64-bit hash. An inital value of 0 is fine.
    \param value A type that std::hash can operate on.
*/
template <typename Hashable>
void hashCombine(uint64_t& hash, const Hashable& value)
{
    hash ^=
        std::hash<Hashable>()(value) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
}

PointViewSet GroupByFilter::run(PointViewPtr inView)
{
    for (PointId idx = 0; idx < inView->size(); idx++)
    {
        uint64_t key = 0;
        for (const auto& dimId : m_dimIds)
        {
            int64_t val = inView->getFieldAs<int64_t>(dimId, idx);
            hashCombine(key, val);
        }

        PointViewPtr& groupView = m_viewMap[key];
        if (!groupView)
            groupView = inView->makeNew();
        groupView->appendPoint(*inView.get(), idx);
    }

    // Transfer grouped pointviews to output set.
    PointViewSet groupedViewSet;
    for (const auto& groupKV : m_viewMap)
        groupedViewSet.insert(groupKV.second);

    return groupedViewSet;
}

} // namespace pdal
