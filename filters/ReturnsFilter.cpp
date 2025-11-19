/******************************************************************************
 * Copyright (c) 2018, Bradley J Chambers (brad.chambers@gmail.com)
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

#include "ReturnsFilter.hpp"

#include <pdal/util/ProgramArgs.hpp>

namespace pdal
{

static StaticPluginInfo const s_info{
    "filters.returns", "Split data by return order",
    "http://pdal.org/stages/filters.returns.html"};

CREATE_STATIC_STAGE(ReturnsFilter, s_info)

std::string ReturnsFilter::getName() const
{
    return s_info.name;
}

void ReturnsFilter::addArgs(ProgramArgs& args)
{
    args.add("groups",
             "Comma-separated list of return number groupings ('first', "
             "'last', 'intermediate', or 'only')",
             m_returnsString, {"last"});
}

void ReturnsFilter::prepared(PointTableRef table)
{
    const PointLayoutPtr layout(table.layout());
    if (!layout->hasDim(Dimension::Id::ReturnNumber) ||
        !layout->hasDim(Dimension::Id::NumberOfReturns))
    {
        log()->get(LogLevel::Warning)
            << "Could not find ReturnNumber or "
               "NumberOfReturns. Proceeding with all returns.\n";
    }
}

PointViewSet ReturnsFilter::run(PointViewPtr inView)
{
    PointViewSet viewSet;

    m_outputTypes = 0;
    for (auto& r : m_returnsString)
    {
        Utils::trim(r);
        if (r == "first")
            m_outputTypes |= returnFirst;
        else if (r == "intermediate")
            m_outputTypes |= returnIntermediate;
        else if (r == "last")
            m_outputTypes |= returnLast;
        else if (r == "only")
            m_outputTypes |= returnOnly;
        else
            throwError("Invalid output type: '" + r + "'.");
    }

    PointViewPtr firstView = inView->makeNew();
    PointViewPtr intermediateView = inView->makeNew();
    PointViewPtr lastView = inView->makeNew();
    PointViewPtr onlyView = inView->makeNew();

    for (PointId idx = 0; idx < inView->size(); idx++)
    {
        PointRef p = inView->point(idx);
        uint8_t rn = p.getFieldAs<uint8_t>(Dimension::Id::ReturnNumber);
        uint8_t nr = p.getFieldAs<uint8_t>(Dimension::Id::NumberOfReturns);
        if ((m_outputTypes & returnFirst) && (rn == 1) && (nr > 1))
            firstView->appendPoint(*inView.get(), idx);
        if ((m_outputTypes & returnIntermediate) && (rn > 1) && (rn < nr) &&
            (nr > 2))
            intermediateView->appendPoint(*inView.get(), idx);
        if ((m_outputTypes & returnLast) && (rn == nr) && (nr > 1))
            lastView->appendPoint(*inView.get(), idx);
        if ((m_outputTypes & returnOnly) && (nr == 1))
            onlyView->appendPoint(*inView.get(), idx);
    }

    if (m_outputTypes & returnFirst)
    {
        if (firstView->size())
            viewSet.insert(firstView);
        else
            log()->get(LogLevel::Warning)
                << "Requested returns group 'first' is empty\n";
    }

    if (m_outputTypes & returnIntermediate)
    {
        if (intermediateView->size())
            viewSet.insert(intermediateView);
        else
            log()->get(LogLevel::Warning)
                << "Requested returns group 'intermediate' is empty\n";
    }

    if (m_outputTypes & returnLast)
    {
        if (lastView->size())
            viewSet.insert(lastView);
        else
            log()->get(LogLevel::Warning)
                << "Requested returns group 'last' is empty\n";
    }

    if (m_outputTypes & returnOnly)
    {
        if (onlyView->size())
            viewSet.insert(onlyView);
        else
            log()->get(LogLevel::Warning)
                << "Requested returns group 'only' is empty\n";
    }

    return viewSet;
}

} // namespace pdal
