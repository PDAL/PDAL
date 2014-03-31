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

#include <pdal/StageBase.hpp>

#include <iostream>

#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>

#include <pdal/Stage.hpp>
#include <pdal/FileUtils.hpp>

namespace pdal
{

std::vector<StageBase*> StageBase::makeVector()
{
    std::vector<StageBase*> v;
    return v;
}

std::vector<StageBase*> StageBase::makeVector(Stage& sref)
{
    Stage* s = &sref;
    StageBase* sb = s;
    std::vector<StageBase*> v;
    v.push_back(sb);
    return v;
}

std::vector<StageBase*> StageBase::makeVector(const std::vector<Stage*>& stages)
{
    std::vector<StageBase*> v;
    BOOST_FOREACH(Stage* stage, stages)
    {
        v.push_back(stage);
    }
    return v;
}

StageBase::StageBase(const std::vector<StageBase*>& inputs, const Options& options)
    : m_initialized(false)
    , m_options(options)
    , m_debug(options.getValueOrDefault<bool>("debug", false))
    , m_verbose(options.getValueOrDefault<boost::uint32_t>("verbose", 0))
    , m_id(options.getValueOrDefault<boost::uint32_t>("id", 0))
    , m_inputs(inputs)
    , m_dimensionsType(StageOperation_All)
    , m_log(LogPtr())
{
    BOOST_FOREACH(StageBase* input, m_inputs)
    {
        input->m_outputs.push_back(this);
    }

    if (m_debug && m_verbose == 0)
        m_verbose = 1;
}


StageBase::~StageBase()
{}

void StageBase::initialize()
{
    // first, initialize any previous stages
    BOOST_FOREACH(StageBase* prev, getInputs())
    {
        prev->initialize();
    }

    // it is illegal to call initialize() twice
    if (m_initialized)
    {
        throw pdal_error("Class already initialized: " + this->getName());
    }

    m_debug = m_options.getValueOrDefault<bool>("debug", false);
    m_verbose = m_options.getValueOrDefault<boost::uint32_t>("verbose", 0);
    if (m_debug && m_verbose == 0)
        m_verbose = 1;

    std::vector<StageBase*> const& inputs = getInputs();
    if (inputs.size() == 0)
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
            LogPtr l = getPrevStage().log();
            std::ostream* v = getPrevStage().log()->getLogStream();
            m_log = boost::shared_ptr<pdal::Log>(new Log(getName(), v));
        }
    }

    m_log->setLevel(static_cast<LogLevel>(m_verbose));

    m_initialized = true;

    return;
}

bool StageBase::isInitialized() const
{
    return m_initialized;
}


const Options& StageBase::getOptions() const
{
    return m_options;
}


Metadata StageBase::getMetadata() const
{
    return m_metadata;
}

Metadata StageBase::collectMetadata() const
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

Options& StageBase::getOptions()
{
    return m_options;
}


bool StageBase::isDebug() const
{
    return m_debug;
}


bool StageBase::isVerbose() const
{
    return m_verbose > 0;
}


boost::uint32_t StageBase::getVerboseLevel() const
{
    return m_verbose;
}


const std::vector<StageBase*>& StageBase::getInputs() const
{
    return m_inputs;
}


const std::vector<StageBase*>& StageBase::getOutputs() const
{
    return m_outputs;
}


Stage& StageBase::getPrevStage() const
{
    // BUG: should probably do this once and cache it
    if (getInputs().size()==0)
        throw internal_error("StageBase does not have any previous stages");
    StageBase* sb = getInputs()[0];
    Stage* s = dynamic_cast<Stage*>(sb);
    if (!s) throw internal_error("previous StageBase is not a Stage");
    return *s;
}


std::vector<Stage*> StageBase::getPrevStages() const
{
    // BUG: should probably do this once and cache it
    std::vector<Stage*> vec;
    BOOST_FOREACH(StageBase* prev, getInputs())
    {
        Stage* s = dynamic_cast<Stage*>(prev);
        if (!s) throw internal_error("previous StageBase is not a Stage");
        vec.push_back(s);
    }
    return vec;
}


std::ostream& operator<<(std::ostream& ostr, const StageBase& stage)
{
    // boost::property_tree::ptree tree = stage.toPTree();
    // 
    // boost::property_tree::write_json(ostr, tree);
    // 
    return ostr;
}


} // namespace pdal
