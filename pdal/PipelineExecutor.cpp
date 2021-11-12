/******************************************************************************
* Copyright (c) 2016, Howard Butler (howard@hobu.co)
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

#include <pdal/PipelineExecutor.hpp>
#include <pdal/PDALUtils.hpp>

namespace pdal
{


PipelineExecutor::PipelineExecutor(std::string const& json)
    : m_json(json)
    , m_executed(false)
    , m_logLevel(pdal::LogLevel::Error)
    , m_pipelineRead(false)
{}


std::string PipelineExecutor::getPipeline() const
{
    if (!m_executed)
        throw pdal_error("Pipeline has not been executed!");

    std::stringstream strm;
    pdal::PipelineWriter::writePipeline(m_manager.getStage(), strm);
    return strm.str();
}


std::string PipelineExecutor::getMetadata() const
{
    if (!m_executed)
        throw pdal_error("Pipeline has not been executed!");

    std::stringstream strm;
    MetadataNode root = m_manager.getMetadata().clone("metadata");
    pdal::Utils::toJSON(root, strm);
    return strm.str();
}


std::string PipelineExecutor::getSchema() const
{
    if (!m_executed)
        throw pdal_error("Pipeline has not been executed!");

    std::stringstream strm;
    MetadataNode root = m_manager.pointTable().layout()->toMetadata().clone("schema");
    pdal::Utils::toJSON(root, strm);
    return strm.str();
}

void PipelineExecutor::readPipeline()
{
    if (!m_pipelineRead)
    {
        std::stringstream strm;
        strm << m_json;
        m_manager.readPipeline(strm);
        m_pipelineRead = true;
    }
}

bool PipelineExecutor::pipelineRead() const
{
    return m_pipelineRead;
}

bool PipelineExecutor::validate()
{
    readPipeline();
    m_manager.prepare();

    return true;
}

int64_t PipelineExecutor::execute()
{
    readPipeline();
    point_count_t count = m_manager.execute();
    m_executed = true;
    return count;
}


void PipelineExecutor::setLogStream(std::ostream& strm)
{

    LogPtr log(Log::makeLog("pypipeline", &strm));
    log->setLevel(m_logLevel);
    m_manager.setLog(log);

}


void PipelineExecutor::setLogLevel(int level)
{
    if (level < 0 || level > 8)
        throw pdal_error("log level must be between 0 and 8!");

    m_logLevel = static_cast<pdal::LogLevel>(level);
    setLogStream(m_logStream);
}


int PipelineExecutor::getLogLevel() const
{
    return static_cast<int>(m_logLevel);
}


std::string PipelineExecutor::getLog() const
{
    return m_logStream.str();
}


} //namespace pdal

