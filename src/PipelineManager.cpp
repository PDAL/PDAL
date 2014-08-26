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

#include <pdal/PipelineManager.hpp>

#include <pdal/Filter.hpp>
#include <pdal/Reader.hpp>
#include <pdal/Writer.hpp>
#include <pdal/Utils.hpp>

#include <boost/property_tree/xml_parser.hpp>
#include <boost/optional.hpp>

namespace pdal
{

PipelineManager::PipelineManager()
    : m_lastStage(NULL)
    , m_lastWriter(NULL)
    , m_isWriterPipeline(false)
{

    return;
}


PipelineManager::~PipelineManager()
{
    while (m_readers.size())
    {
        Reader* reader = m_readers.back();
        m_readers.pop_back();
        delete reader;
    }

    while (m_filters.size())
    {
        Filter* filter = m_filters.back();
        m_filters.pop_back();
        delete filter;
    }

    while (m_writers.size())
    {
        Writer* writer = m_writers.back();
        m_writers.pop_back();
        delete writer;
    }

    return;
}

void PipelineManager::removeWriter()
{
    while (m_writers.size())
    {
        Writer* writer = m_writers.back();
        m_writers.pop_back();
        delete writer;
    }

    m_lastWriter = 0;
}

Reader* PipelineManager::addReader(const std::string& type,
    const Options& options)
{
    registerPluginIfExists(options);

    Reader* stage = m_factory.createReader(type, options);
    m_readers.push_back(stage);
    m_lastStage = stage;
    return stage;
}


Filter* PipelineManager::addFilter(const std::string& type,
    const std::vector<Stage *>& prevStages, const Options& options)
{
    registerPluginIfExists(options);

    Filter* stage = m_factory.createFilter(type, options);
    stage->setInput(prevStages);
    m_filters.push_back(stage);
    m_lastStage = stage;
    return stage;
}


Filter* PipelineManager::addFilter(const std::string& type, Stage *prevStage,
    const Options& options)
{
    registerPluginIfExists(options);

    Filter* stage = m_factory.createFilter(type, options);
    stage->setInput(prevStage);
    m_filters.push_back(stage);
    m_lastStage = stage;
    return stage;
}


void PipelineManager::registerPluginIfExists(const Options& options)
{
    if (options.hasOption("plugin"))
    {
        m_factory.registerPlugin(options.getValueOrThrow<std::string>("plugin"));
    }
}


Writer* PipelineManager::addWriter(const std::string& type, Stage *prevStage,
    const Options& options)
{
    m_isWriterPipeline = true;

    registerPluginIfExists(options);

    Writer* writer = m_factory.createWriter(type, options);
    writer->setInput(prevStage);
    m_writers.push_back(writer);
    m_lastWriter = writer;
    return writer;
}


Writer* PipelineManager::getWriter() const
{
    return m_lastWriter;
}


Stage* PipelineManager::getStage() const
{
    return m_lastStage;
}


point_count_t PipelineManager::execute()
{
    m_lastStage->prepare(m_context);
    m_pbSet = m_lastStage->execute(m_context);
    point_count_t cnt = 0;
    for (auto pi = m_pbSet.begin(); pi != m_pbSet.end(); ++pi)
    {
        PointBufferPtr buf = *pi;
        cnt += buf->size();
    }
    return cnt;
}


bool PipelineManager::isWriterPipeline() const
{
    return (m_lastWriter != NULL);
}

MetadataNode PipelineManager::getMetadata() const
{
    MetadataNode output("stages");
    
    for (auto ri = m_readers.begin(); ri != m_readers.end(); ++ri)
        output.add((*ri)->getMetadata());
    
    for (auto fi = m_filters.begin(); fi != m_filters.end(); ++fi)
        output.add((*fi)->getMetadata());

    for (auto wi = m_writers.begin(); wi != m_writers.end(); ++wi)
        output.add((*wi)->getMetadata());
        
    return output;
}
} // namespace pdal
