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

namespace pdal
{

Streamable::Streamable()
{}


bool Streamable::pipelineStreamable() const
{
    for (const Stage *s : m_inputs)
        if (!s->pipelineStreamable())
            return false;
    return true;
}


// Streamed execution.
void Streamable::execute(StreamPointTable& table)
{
    m_log->get(LogLevel::Debug) << "Executing pipeline in stream mode." <<
        std::endl;
    struct StreamableList : public std::list<Streamable *>
    {
        StreamableList operator - (const StreamableList& other) const
        {
            StreamableList resultList;
            auto ti = rbegin();
            auto oi = other.rbegin();

            while (oi != other.rend() && ti != rend() && *ti == *oi)
            {
                oi++;
                ti++;
            }
            while (ti != rend())
                resultList.push_front(*ti++);
            return resultList;
        };

        void ready(PointTableRef& table)
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

        void done(PointTableRef& table)
        {
            for (auto s : *this)
            {
                s->startLogging();
                s->l_done(table);
                s->stopLogging();
            }
        }
    };

    if (!pipelineStreamable())
        throwError("Attempting to use stream mode with a stage that doesn't "
            "support streaming.");

    SpatialReference srs;
    std::list<StreamableList> lists;
    StreamableList stages;
    StreamableList lastRunStages;

    table.finalize();

    // Walk from the current stage backwards.  As we add each input, copy
    // the list of stages and push it on a list.  We then pull a list from the
    // back of list and keep going.  Pushing on the front and pulling from the
    // back insures that the stages will be executed in the order that they
    // were added.  If we hit stage with no previous stages, we execute
    // the stage list.
    // All this often amounts to a bunch of list copying for
    // no reason, but it's more simple than what we might otherwise do and
    // this should be a nit in the grand scheme of execution time.
    //
    // As an example, if there are four paths from the end stage (writer) to
    // reader stages, there will be four stage lists and execute(table, stages)
    // will be called four times.
    SrsMap srsMap;
    Streamable *s = this;
    stages.push_front(s);
    while (true)
    {
        if (s->m_inputs.empty())
        {
            // Call done on all the stages we ran last time and aren't
            // using this time.
            (lastRunStages - stages).done(table);
            // Call ready on all the stages we didn't run last time.
            (stages - lastRunStages).ready(table);
            execute(table, stages, srsMap);
            lastRunStages = stages;
        }
        else
        {
            for (auto s2 : s->m_inputs)
            {
                StreamableList newStages(stages);
                newStages.push_front(dynamic_cast<Streamable *>(s2));
                lists.push_front(newStages);
            }
        }
        if (lists.empty())
        {
            lastRunStages.done(table);
            break;
        }
        stages = lists.back();
        lists.pop_back();
        s = stages.front();
    }
}


void Streamable::execute(StreamPointTable& table,
    std::list<Streamable *>& stages, SrsMap& srsMap)
{
    std::vector<bool> skips(table.capacity());
    std::list<Streamable *> filters;
    SpatialReference srs;

    // Separate out the first stage.
    Streamable *reader = stages.front();

    // Build a list of all stages except the first.  We may have a writer in
    // this list in addition to filters, but we treat them in the same way.
    auto begin = stages.begin();
    begin++;
    std::copy(begin, stages.end(), std::back_inserter(filters));

    // Loop until we're finished.  We handle the number of points up to
    // the capacity of the StreamPointTable that we've been provided.

    bool finished = false;
    while (!finished)
    {
        // Clear the spatial reference when processing starts.
        table.clearSpatialReferences();
        PointId idx = 0;
        PointRef point(table, idx);
        point_count_t pointLimit = table.capacity();

        reader->startLogging();
        // When we get false back from a reader, we're done, so set
        // the point limit to the number of points processed in this loop
        // of the table.
        if (!pointLimit)
            finished = true;

        for (PointId idx = 0; idx < pointLimit; idx++)
        {
            point.setPointId(idx);
            finished = !reader->processOne(point);
            if (finished)
                pointLimit = idx;
        }
        reader->stopLogging();
        srs = reader->getSpatialReference();
        if (!srs.empty())
            table.setSpatialReference(srs);

        // When we get a false back from a filter, we're filtering out a
        // point, so add it to the list of skips so that it doesn't get
        // processed by subsequent filters.
        for (Streamable *s : filters)
        {
            if (srsMap[s] != srs)
            {
                s->spatialReferenceChanged(srs);
                srsMap[s] = srs;
            }
            s->startLogging();
            for (PointId idx = 0; idx < pointLimit; idx++)
            {
                if (skips[idx])
                    continue;
                point.setPointId(idx);
                if (!s->processOne(point))
                    skips[idx] = true;
            }
            srs = s->getSpatialReference();
            if (!srs.empty())
                table.setSpatialReference(srs);
            s->stopLogging();
        }

        // Yes, vector<bool> is terrible.  Can do something better later.
        for (size_t i = 0; i < skips.size(); ++i)
            skips[i] = false;
        table.reset();
    }
}

} // namespace pdal

