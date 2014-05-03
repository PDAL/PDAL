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

#include <pdal/Stage.hpp>
#include <pdal/SpatialReference.hpp>

#include <boost/concept_check.hpp> // ignore_unused_variable_warning
#include <boost/foreach.hpp>

namespace pdal
{

Stage::Stage(PointContext context, const Options& options) :
    Stage{context}
{
    m_options = options;
    m_debug = m_options.getValueOrDefault<bool>("debug", false);
    m_verbose = m_options.getValueOrDefault<boost::uint32_t>("verbose", 0);
    m_id = m_options.getValueOrDefault<boost::uint32_t>("id", 0);
    if (m_debug && !m_verbose)
        m_verbose = 1;
}


Stage::Stage(PointContext context) :
    m_context(context), m_initialized(false), m_debug(false), m_verbose(0),
    m_id(0), m_dimensionsType(StageOperation_All),
    m_log(LogPtr()), m_numPoints(0)
{}

void Stage::setInput(const std::vector<Stage *>& inputs)
{
    m_inputs = inputs;
    for (size_t i = 0; i < m_inputs.size(); ++i)
    {
        Stage *input = m_inputs[i];
        input->m_outputs.push_back(this);
    }
}

void Stage::setInput(Stage *input)
{
    m_inputs.push_back(input);
    input->m_outputs.push_back(this);
}


void Stage::initialize()
{

    for (size_t i = 0; i < m_inputs.size(); ++i)
    {
        Stage *prev = m_inputs[i];
        prev->initialize();
    }

    // it is illegal to call initialize() twice
    if (m_initialized)
        throw pdal_error("Class already initialized: " + this->getName());

    m_debug = m_options.getValueOrDefault<bool>("debug", false);
    m_verbose = m_options.getValueOrDefault<boost::uint32_t>("verbose", 0);
    if (m_debug && !m_verbose)
        m_verbose = 1;

    if (m_inputs.empty())
    {
        std::string logname =
            m_options.getValueOrDefault<std::string>("log", "stdlog");
        m_log = boost::shared_ptr<pdal::Log>(new Log(getName(), logname));
    }
    else
    {
        if (m_options.hasOption("log"))
        {
            std::string logname = m_options.getValueOrThrow<std::string>("log");
            m_log = boost::shared_ptr<pdal::Log>(new Log(getName(), logname));
        }
        else
        {
            std::ostream* v = getPrevStage().log()->getLogStream();
            m_log = boost::shared_ptr<pdal::Log>(new Log(getName(), v));
        }
    }
    m_log->setLevel((LogLevel)m_verbose);
    m_initialized = true;

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
}


Metadata Stage::collectMetadata() const
{
    Metadata output(m_metadata);
    output = output + m_metadata;

    try
    {
        Metadata const& m = getPrevStage().getMetadata();
        output = output + m;
    }
    catch (pdal::internal_error const&)
    {
    }
    return output;
}


Stage& Stage::getPrevStage() const
{
    if (m_inputs.empty())
        throw internal_error("Stage does not have any previous stages");
    return *m_inputs[0];
}


std::vector<Stage*> Stage::getPrevStages() const
{
    return m_inputs;
}


const Bounds<double>& Stage::getBounds() const
{
    return m_bounds;
}


void Stage::setBounds(const Bounds<double>& bounds)
{
    m_bounds = bounds;
}


void Stage::setSchema(const Schema& schema)
{
    m_schema = schema;
}


boost::uint64_t Stage::getNumPoints() const
{
    // The Stage's getNumPoints() can't change. If it is 0, we'll try to
    // forward the getPrevStage()'s count. This will continue down the
    // pipeline until a count is returned. If the end stage does
    // actually return a 0 because the Stage has an unknown or indefinite
    // point count, this will ask and return 0 every time.

    // We will cache this value on the stage once we've returned
    // it because the point count of a stage is not expected to change.
    // m_numPoints is mutable to support the setting of its value
    // to cache the previous stage's value.
    if (m_numPoints == 0)
    {
        try
        {
            m_numPoints = getPrevStage().getNumPoints();
        }
        catch (pdal::internal_error const&)
        {}
    }
    return m_numPoints;
}


void Stage::setNumPoints(boost::uint64_t numPoints)
{
    m_numPoints = numPoints;
}


const SpatialReference& Stage::getSpatialReference() const
{
    return m_spatialReference;
}


void Stage::setSpatialReference(const SpatialReference& spatialReference)
{
    m_spatialReference = spatialReference;


    boost::optional<SpatialReference> ref =
        m_metadata.getValueOptional<SpatialReference>("spatialreference");
    if (!ref)
    {
        m_metadata.deleteMetadata("spatialreference");
        m_metadata.addMetadata<pdal::SpatialReference>("spatialreference",
                spatialReference,
                "SRS of this stage");
    }
}

void Stage::setCoreProperties(const Stage& stage)
{
    this->setSchema(stage.getSchema());
    this->setNumPoints(stage.getNumPoints());
    this->setBounds(stage.getBounds());
    this->setSpatialReference(stage.getSpatialReference());
}

std::vector<Stage *> Stage::makeVector(Stage& sref)
{
    std::vector<Stage *> v;
    v.push_back(&sref);
    return v;
}

std::vector<Stage*> Stage::makeVector(const std::vector<Stage*>& stages)
{
    return stages;
}

std::ostream& operator<<(std::ostream& ostr, const Stage& stage)
{
    ostr << "  Name: " << stage.getName() << std::endl;
    ostr << "  Num points: " << stage.getNumPoints() << std::endl;

    ostr << "  Bounds:" << std::endl;
    ostr << "    " << stage.getBounds() << std::endl;

    ostr << "  Schema: " << std::endl;
    ostr << "    Num dims: " << stage.getSchema().getDimensions().size() <<
        std::endl;

    ostr << "  Spatial Reference:" << std::endl;
    ostr << "    WKT: " << stage.getSpatialReference().getWKT() << std::endl;

    return ostr;
}

} // namespace pdal
