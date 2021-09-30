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


PipelineExecutor::PipelineExecutor(std::string const& json, point_count_t streamLimit)
    : m_managerPtr(new pdal::PipelineManager(streamLimit))
    , m_executed(false)
{
    std::stringstream strm;
    strm << json;
    m_managerPtr->readPipeline(strm);
}


std::string PipelineExecutor::getPipeline() const
{
    if (!m_executed)
        throw pdal_error("Pipeline has not been executed!");

    std::stringstream strm;
    pdal::PipelineWriter::writePipeline(m_managerPtr->getStage(), strm);
    return strm.str();
}


std::string PipelineExecutor::getMetadata() const
{
    if (!m_executed)
        throw pdal_error("Pipeline has not been executed!");

    std::stringstream strm;
    MetadataNode root = m_managerPtr->getMetadata().clone("metadata");
    pdal::Utils::toJSON(root, strm);
    return strm.str();
}


std::string PipelineExecutor::getSchema() const
{
    if (!m_executed)
        throw pdal_error("Pipeline has not been executed!");

    std::stringstream strm;
    MetadataNode root = m_managerPtr->pointTable().layout()->toMetadata().clone("schema");
    pdal::Utils::toJSON(root, strm);
    return strm.str();
}


const PointViewSet& PipelineExecutor::views() const {
    if (!m_executed)
        throw pdal_error("Pipeline has not been executed!");

    return m_managerPtr->views();
}


bool PipelineExecutor::validate()
{
    m_managerPtr->prepare();
    return true;
}


point_count_t PipelineExecutor::execute()
{
    point_count_t count = m_managerPtr->execute();
    m_executed = true;
    return count;
}


void PipelineExecutor::setLogLevel(int level)
{
    if (level < 0 || level > 8)
        throw pdal_error("log level must be between 0 and 8!");

    std::ostream* logStream;
    if (m_managerPtr->log())
        logStream = m_managerPtr->log()->getLogStream();
    else
        logStream = std::shared_ptr<std::ostream>(new std::stringstream()).get();
    LogPtr log(Log::makeLog("pypipeline", logStream));
    log->setLevel(static_cast<pdal::LogLevel>(level));
    m_managerPtr->setLog(log);
}


int PipelineExecutor::getLogLevel() const
{
    auto level = (m_managerPtr->log() ? m_managerPtr->log()->getLevel()
                                      : pdal::LogLevel::Error);
    return static_cast<int>(level);
}


std::string PipelineExecutor::getLog() const
{
    if (!m_managerPtr->log())
        return "";
    auto logStream = m_managerPtr->log()->getLogStream();
    return static_cast<std::stringstream*>(logStream)->str();
}


point_count_t PipelineStreamableExecutor::execute()
{
    point_count_t count = 0;
    while (PointViewPtr view = executeNext())
        count += view->size();
    return count;
}


PointViewPtr PipelineStreamableExecutor::executeNext()
{
    if (!m_itPtr)
        m_itPtr = std::unique_ptr<StreamableIterator>(m_managerPtr->executeStream());

    StreamableIterator& it = *m_itPtr;
    if (!it) {
        m_executed = true;
        return nullptr;
    }
    ++it;
    return *it;
}


} //namespace pdal
