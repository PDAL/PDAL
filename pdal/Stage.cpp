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

#include <pdal/GDALUtils.hpp>
#include <pdal/GEOSUtils.hpp>
#include <pdal/PipelineManager.hpp>
#include <pdal/Stage.hpp>
#include <pdal/SpatialReference.hpp>
#include <pdal/PDALUtils.hpp>
#include <pdal/util/Algorithm.hpp>
#include <pdal/util/ProgramArgs.hpp>

#include "private/StageRunner.hpp"

#include <iterator>
#include <memory>

namespace pdal
{

Stage::Stage() : m_progressFd(-1), m_debug(false), m_verbose(0),
    m_pointCount(0), m_faceCount(0)
{}


void Stage::addConditionalOptions(const Options& opts)
{
    for (const auto& o : opts.getOptions())
        m_options.addConditional(o);
}


void Stage::serialize(MetadataNode root, PipelineWriter::TagMap& tags) const
{
    for (Stage *s : m_inputs)
        s->serialize(root, tags);

    auto tagname = [tags](const Stage *s)
    {
        const auto ti = tags.find(s);
        return ti->second;
    };

    MetadataNode anon("pipeline");
    anon.add("type", getName());
    anon.add("tag", tagname(this));
    m_options.toMetadata(anon);
    for (Stage *s : m_inputs)
        anon.addList("inputs", tagname(s));
    root.addList(anon);
}

void Stage::addAllArgs(ProgramArgs& args)
{
    try
    {
        l_addArgs(args);
        addArgs(args);
    }
    catch (arg_error error)
    {
        throw pdal_error(getName() + ": " + error.m_error);
    }
}


void Stage::handleOptions()
{
    addAllArgs(*m_args);
    try
    {
        StringList cmdline = m_options.toCommandLine();
        m_args->parse(cmdline);
    }
    catch (arg_error error)
    {
        throw pdal_error(getName() + ": " + error.m_error);
    }
    setupLog();
}


QuickInfo Stage::preview()
{
    m_args.reset(new ProgramArgs);
    handleOptions();
    pushLogLeader();
    QuickInfo qi = inspect();
    popLogLeader();
    return qi;
}


void Stage::prepare(PointTableRef table)
{
    m_args.reset(new ProgramArgs);
    for (size_t i = 0; i < m_inputs.size(); ++i)
    {
        Stage *prev = m_inputs[i];
        prev->prepare(table);
    }
    handleOptions();
    pushLogLeader();
    l_initialize(table);
    initialize(table);
    addDimensions(table.layout());
    prepared(table);
    popLogLeader();
}


PointViewSet Stage::execute(PointTableRef table)
{
    pushLogLeader();
    table.finalize();

    PointViewSet views;

    // If the inputs are empty, we're a reader.
    if (m_inputs.empty())
    {
        views.insert(PointViewPtr(new PointView(table)));
    }
    else
    {
        for (size_t i = 0; i < m_inputs.size(); ++i)
        {
            Stage *prev = m_inputs[i];
            PointViewSet temp = prev->execute(table);
            views.insert(temp.begin(), temp.end());
        }
    }

    PointViewSet outViews;
    std::vector<StageRunnerPtr> runners;

    // Put the spatial references from the views onto the table.
    // The table's spatial references are only valid as long as the stage
    // is running.
    // ABELL - Should we clear the references once the stage run has
    //   completed?  Wondering if that would break something where a
    //   writer wants to check a table's SRS.
    SpatialReference srs;
    table.clearSpatialReferences();
    // Iterating backwards will ensure that the SRS for the first view is
    // first on the list for table.
    for (auto it = views.rbegin(); it != views.rend(); it++)
        table.addSpatialReference((*it)->spatialReference());
    gdal::ErrorHandler::getGlobalErrorHandler().set(m_log, m_debug);

    // Count the number of views and the number of points and faces so they're
    // available to stages.
    m_pointCount = 0;
    m_faceCount = 0;
    for (auto const& it : views)
    {
        m_pointCount += it->size();
        auto m = it->mesh();
        if (m)
            m_faceCount += m->size();
    }
    // Do the ready operation and then start running all the views
    // through the stage.
    ready(table);
    for (auto const& it : views)
    {
        StageRunnerPtr runner(new StageRunner(this, it));
        runners.push_back(runner);
        runner->run();
    }

    // As the stages complete (synchronously at this time), propagate the
    // spatial reference and merge the output views.
    srs = getSpatialReference();
    for (auto const& it : runners)
    {
        StageRunnerPtr runner(it);
        PointViewSet temp = runner->wait();

        // If our stage has a spatial reference, the view takes it on once
        // the stage has been run.
        if (!srs.empty())
            for (PointViewPtr v : temp)
                v->setSpatialReference(srs);
        outViews.insert(temp.begin(), temp.end());
    }
    l_done(table);
    popLogLeader();
    m_pointCount = 0;
    m_faceCount = 0;
    return outViews;
}


// Streamed execution.
void Stage::execute(StreamPointTable& table)
{
    struct StageList : public std::list<Stage *>
    {
        StageList operator - (const StageList& other) const
        {
            StageList resultList;
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
                s->pushLogLeader();
                s->ready(table);
                s->pushLogLeader();
                SpatialReference srs = s->getSpatialReference();
                if (!srs.empty())
                    table.setSpatialReference(srs);
            }
        }

        void done(PointTableRef& table)
        {
            for (auto s : *this)
            {
                s->pushLogLeader();
                s->l_done(table);
                s->popLogLeader();
            }
        }
    };

    SpatialReference srs;
    std::list<StageList> lists;
    StageList stages;
    StageList lastRunStages;

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
    Stage *s = this;
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
            execute(table, stages);
            lastRunStages = stages;
        }
        else
        {
            for (auto s2 : s->m_inputs)
            {
                StageList newStages(stages);
                newStages.push_front(s2);
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


void Stage::execute(StreamPointTable& table, std::list<Stage *>& stages)
{
    std::vector<bool> skips(table.capacity());
    std::list<Stage *> filters;
    SpatialReference srs;
    std::map<Stage *, SpatialReference> srsMap;

    // Separate out the first stage.
    Stage *reader = stages.front();

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

        reader->pushLogLeader();
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
        reader->popLogLeader();
        srs = reader->getSpatialReference();
        if (!srs.empty())
            table.setSpatialReference(srs);

        // When we get a false back from a filter, we're filtering out a
        // point, so add it to the list of skips so that it doesn't get
        // processed by subsequent filters.
        for (Stage *s : filters)
        {
            if (srsMap[s] != srs)
            {
                s->spatialReferenceChanged(srs);
                srsMap[s] = srs;
            }
            s->pushLogLeader();
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
            s->popLogLeader();
        }

        // Yes, vector<bool> is terrible.  Can do something better later.
        for (size_t i = 0; i < skips.size(); ++i)
            skips[i] = false;
        table.reset();
    }
}

void Stage::l_done(PointTableRef table)
{
    done(table);
}

void Stage::l_addArgs(ProgramArgs& args)
{
    args.add("user_data", "User JSON", m_userDataJSON);
    args.add("log", "Debug output filename", m_logname);
    readerAddArgs(args);
}


void Stage::setupLog()
{
    LogLevel l(LogLevel::Error);

    if (m_log)
    {
        l = m_log->getLevel();
        m_logLeader = m_log->leader();
    }

    if (!m_logname.empty())
        m_log.reset(new Log("", m_logname));
    else if (!m_log)
        m_log.reset(new Log("", "stdlog"));
    m_log->setLevel(l);

    // Add the stage name to the existing leader.
    if (m_logLeader.size())
        m_logLeader += " ";
    m_logLeader += getName();

    bool debug(l > LogLevel::Debug);
    gdal::ErrorHandler::getGlobalErrorHandler().set(m_log, debug);
}


void Stage::l_initialize(PointTableRef table)
{
    m_metadata = table.metadata().add(getName());
    writerInitialize(table);
}


void Stage::addSpatialReferenceArg(ProgramArgs& args)
{
    args.add("spatialreference", "Spatial reference to apply to data",
        m_spatialReference);
}

const SpatialReference& Stage::getSpatialReference() const
{
    return m_spatialReference;
}


void Stage::setSpatialReference(const SpatialReference& spatialRef)
{
    setSpatialReference(m_metadata, spatialRef);
}


void Stage::setSpatialReference(MetadataNode& m,
    const SpatialReference& spatialRef)
{
    m_spatialReference = spatialRef;

    auto pred = [](MetadataNode m){ return m.name() == "spatialreference"; };

    MetadataNode spatialNode = m.findChild(pred);
    if (spatialNode.empty())
    {
        m.add(spatialRef.toMetadata());
        m.add("spatialreference", spatialRef.getWKT(), "SRS of this stage");
        m.add("comp_spatialreference", spatialRef.getWKT(),
            "SRS of this stage");
    }
}


bool Stage::parseName(std::string o, std::string::size_type& pos)
{
    auto isStageChar = [](char c)
        { return std::islower(c) || std::isdigit(c); };

    std::string::size_type start = pos;
    if (!std::islower(o[pos]))
        return false;
    pos++;
    pos += Utils::extract(o, pos, isStageChar);
    return true;
}


bool Stage::parseTagName(std::string o, std::string::size_type& pos)
{
    auto isTagChar = [](char c)
        { return std::isalnum(c) || c == '_'; };

    std::string::size_type start = pos;
    if (!std::isalpha(o[pos]))
        return false;
    pos++;
    pos += Utils::extract(o, pos, isTagChar);
    return true;
}


void Stage::throwError(const std::string& s) const
{
    throw pdal_error(getName() + ": " + s);
}

} // namespace pdal

