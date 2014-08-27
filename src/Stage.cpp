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

#include <pdal/GlobalEnvironment.hpp>
#include <pdal/Stage.hpp>
#include <pdal/SpatialReference.hpp>
#include <pdal/StageRunner.hpp>

namespace pdal
{


Stage::Stage(const Options& options)
{
    Construct();
    m_options = options;
    m_debug = m_options.getValueOrDefault<bool>("debug", false);
    m_verbose = m_options.getValueOrDefault<boost::uint32_t>("verbose", 0);
    if (m_debug && !m_verbose)
        m_verbose = 1;
}


Stage::Stage()
{
    Construct();
}


void Stage::Construct()
{
    m_debug = false;
    m_verbose = 0;
}


void Stage::prepare(PointContext ctx)
{
    for (size_t i = 0; i < m_inputs.size(); ++i)
    {
        Stage *prev = m_inputs[i];
        prev->prepare(ctx);
    }
    l_processOptions(m_options);
    processOptions(m_options);
    l_initialize(ctx);
    initialize();
    addDimensions(ctx);
}


PointBufferSet Stage::execute(PointContext ctx)
{
    PointBufferSet buffers;
    if (m_inputs.empty())
        buffers.insert(PointBufferPtr(new PointBuffer(ctx)));
    else
    {
        for (size_t i = 0; i < m_inputs.size(); ++i)
        {
            Stage *prev = m_inputs[i];
            PointBufferSet temp = prev->execute(ctx);
            buffers.insert(temp.begin(), temp.end());
        }
    }

    PointBufferSet outBuffers;
    std::vector<StageRunnerPtr> runners;

    ready(ctx);
    for (auto it = buffers.begin(); it != buffers.end(); ++it)
    {
        StageRunnerPtr runner(new StageRunner(this, *it));
        runners.push_back(runner);
        runner->run();
    }
    for (auto it = runners.begin(); it != runners.end(); ++it)
    {
        StageRunnerPtr runner(*it);
        PointBufferSet temp = runner->wait();
        outBuffers.insert(temp.begin(), temp.end());
    }
    l_done(ctx);
    done(ctx);
    return outBuffers;
}


void Stage::l_initialize(PointContext ctx)
{
    m_metadata = ctx.metadata().add(getName());
    if (m_inputs.size()) {
        Stage& prevStage = *m_inputs[0];
    }
}


void Stage::l_processOptions(const Options& options)
{
    if (m_debug && !m_verbose)
        m_verbose = 1;

    if (m_inputs.empty())
    {
        std::string logname =
            options.getValueOrDefault<std::string>("log", "stdlog");
        m_log = boost::shared_ptr<pdal::Log>(new Log(getName(), logname));
    }
    else
    {
        if (options.hasOption("log"))
        {
            std::string logname = m_options.getValueOrThrow<std::string>("log");
            m_log.reset(new Log(getName(), logname));
        }
        else
        {
            // We know we're not empty at this point
            std::ostream* v = m_inputs[0]->log()->getLogStream();
            m_log.reset(new Log(getName(), v));
        }
    }
    m_log->setLevel((LogLevel::Enum)m_verbose);

    // If the user gave us an SRS via options, take that.
    try
    {
        m_spatialReference = getOptions().
            getValueOrThrow<pdal::SpatialReference>("spatialreference");
    }
    catch (pdal_error const&)
    {
        // If one wasn't set on the options, we'll ignore at this
        // point.  Maybe another stage might forward/set it later.
    }

    // Process writer-specific options.
    writerProcessOptions(options);
}


void Stage::l_done(PointContext ctx)
{
    if (!m_spatialReference.empty())
        ctx.setSpatialRef(m_spatialReference);
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
        m.add("spatialreference", spatialRef, "SRS of this stage");
}

std::vector<Stage*> Stage::findStage(std::string name)
{
    std::vector<Stage*> output;
    if (boost::iequals(getName(), name))
        output.push_back(this);
    
    for (auto s = m_inputs.begin(); s != m_inputs.end(); ++s)
    {
        Stage* stage = (*s);
        if (boost::iequals(stage->getName(), name))
            output.push_back(stage);
        if (stage->getInputs().size())
        {
            auto hits = stage->findStage(name);
            if (hits.size())
                output.insert(output.end(), hits.begin(), hits.end());
        }
    }
    
    return output;
}

std::ostream& operator<<(std::ostream& ostr, const Stage& stage)
{
    ostr << "  Name: " << stage.getName() << std::endl;
    ostr << "  Spatial Reference:" << std::endl;
    ostr << "    WKT: " << stage.getSpatialReference().getWKT() << std::endl;

    return ostr;
}

} // namespace pdal
