/******************************************************************************
* Copyright (c) 2011, Michael P. Gerlek (mpg@flaxen.com)
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

#include <iterator>

#include <pdal/Streamable.hpp>
#include <pdal/Filter.hpp>
#include <pdal/Reader.hpp>
#include "../filters/private/expr/ConditionalExpression.hpp"

namespace pdal
{


bool Streamable::pipelineStreamable() const
{
    for (const Stage *s : m_inputs)
        if (!s->pipelineStreamable())
            return false;
    return true;
}


void Streamable::execute(StreamPointTable& table)
{
    for (auto it = executeStream(table); it; ++it);
}

Streamable::Iterator Streamable::executeStream(StreamPointTable& table)
{
    assertStreamable();
    m_log->get(LogLevel::Debug)
        << "Executing pipeline in stream mode." << std::endl;
    table.finalize();
    return Iterator(table, *this);
}

void Streamable::Iterator::populateLists(Streamable* stage,
                                         Streamable::List& currentStages)
{
    currentStages.push_front(stage);
    if (stage->m_inputs.empty())
        lists.push_front(currentStages);
    else
        for (auto bi = stage->m_inputs.rbegin(); bi != stage->m_inputs.rend(); bi++)
        {
            Streamable::List newStages(currentStages);
            populateLists(dynamic_cast<Streamable*>(*bi), newStages);
        }
}

PointViewPtr Streamable::Iterator::operator*() const
{
    auto view = new PointView(table, table.spatialReference());
    for (PointId idx = 0; idx < lastReadCount; idx++)
        if (!table.skip(idx))
            view->appendPoint(idx);
    return PointViewPtr(view);
}

Streamable::Iterator& Streamable::Iterator::operator++()
{
    if (!*this)
        throw pdal_error("Cannot forward Streamable::Iterator past the end");

    Streamable::List& stages = lists.front();
    if (!toReadCount)
    {
        // Call done on all the stages we ran last time and aren't using this time.
        (lastRunStages - stages).done(table);
        // Call ready on all the stages we didn't run last time.
        (stages - lastRunStages).ready(table);
        // The first stage should be a reader.
        Reader* r = dynamic_cast<Reader*>(stages.front());
        // We may be limited in the number of points requested.
        toReadCount = r ? r->count() : (std::numeric_limits<point_count_t>::max)();
    }
    toReadCount = execute(stages, toReadCount);
    if (!toReadCount)
    {
        lastRunStages = stages;
        lists.pop_front();
        if (lists.empty())
            lastRunStages.done(table);
    }
    return *this;
}

point_count_t Streamable::Iterator::execute(Streamable::List& stages,
                                            point_count_t count)
{
    // Clear any previous read points and the spatial reference when processing starts.
    table.clear(lastReadCount);
    table.clearSpatialReferences();

    PointRef point(table);
    point_count_t pointLimit = (std::min)(count, table.capacity());

    auto it = stages.begin();
    Streamable* s = *it++;

    s->startLogging();
    for (PointId idx = 0; idx < pointLimit; idx++)
    {
        point.setPointId(idx);
        // When we get false back from a reader, we're done, so set the point
        // limit to the number of points processed in this loop of the table.
        if (!s->processOne(point))
            pointLimit = idx;
    }
    s->stopLogging();

    SpatialReference srs = s->getSpatialReference();
    if (!srs.empty())
        table.setSpatialReference(srs);

    // Iterate over all stages except the first.  We may have a writer in
    // addition to filters, but we treat them in the same way.
    // When we get a false back from a filter, we're filtering out a
    // point, so add it to the list of skips so that it doesn't get
    // processed by subsequent filters.
    while (it != stages.end())
    {
        s = *it++;
        auto si = srsMap.find(s);
        if (si == srsMap.end() || si->second != srs)
        {
            s->spatialReferenceChanged(srs);
            srsMap[s] = srs;
        }
        s->startLogging();

        const expr::ConditionalExpression* where = s->whereExpr();
        for (PointId idx = 0; idx < pointLimit; idx++)
        {
            point.setPointId(idx);
            if (table.skip(idx))
                continue;
            if (where && !where->eval(point))
                continue;
            if (!s->processOne(point))
                table.setSkip(idx);
        }
        const SpatialReference& tempSrs = s->getSpatialReference();
        if (!tempSrs.empty())
        {
            srs = tempSrs;
            table.setSpatialReference(srs);
        }
        s->stopLogging();
    }

    lastReadCount = pointLimit;
    return pointLimit == table.capacity() ? count - pointLimit : 0;
}

Streamable::List Streamable::List::operator-(const Streamable::List& other) const
{
    auto ti = rbegin();
    auto oi = other.rbegin();
    while (oi != other.rend() && ti != rend() && *ti == *oi)
    {
        oi++;
        ti++;
    }
    Streamable::List resultList;
    while (ti != rend())
        resultList.push_front(*ti++);
    return resultList;
};

void Streamable::List::ready(PointTableRef& table)
{
    for (auto s : *this)
    {
        s->startLogging();
        s->ready(table);
        s->stopLogging();
        SpatialReference srs = s->getSpatialReference();
        if (!srs.empty())
            table.setSpatialReference(srs);
    }
}

void Streamable::List::done(PointTableRef& table)
{
    for (auto s : *this)
    {
        s->startLogging();
        s->done(table);
        s->stopLogging();
    }
}

} // namespace pdal
