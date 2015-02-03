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

#include <pdal/Utils.hpp>

//#include <boost/optional.hpp>

namespace pdal
{

PipelineManager::~PipelineManager()
{
    while (m_stages.size())
    {
        Stage* stage = m_stages.back();
        m_stages.pop_back();
        delete stage;
    }
}


Stage* PipelineManager::addReader(const std::string& type)
{
    Stage *r = m_factory.createStage2(type);
    m_stages.push_back(r);
    return r;
}


Stage* PipelineManager::addFilter(const std::string& type,
    const std::vector<Stage *>& prevStages)
{
    Stage* stage = m_factory.createStage2(type);
    stage->setInput(prevStages);
    m_stages.push_back(stage);
    return stage;
}


Stage* PipelineManager::addFilter(const std::string& type, Stage *prevStage)
{
    Stage* stage = m_factory.createStage2(type);
    stage->setInput(prevStage);
    m_stages.push_back(stage);
    return stage;
}


Stage* PipelineManager::addWriter(const std::string& type, Stage *prevStage)
{
    Stage* writer = m_factory.createStage2(type);
    writer->setInput(prevStage);
    m_stages.push_back(writer);
    return writer;
}


void PipelineManager::prepare() const
{
    Stage *s = getStage();
    if (s)
       s->prepare(m_context);
}


point_count_t PipelineManager::execute()
{
    prepare();

    Stage *s = getStage();
    if (!s)
        return 0;
    m_pbSet = s->execute(m_context);
    point_count_t cnt = 0;
    for (auto pi = m_pbSet.begin(); pi != m_pbSet.end(); ++pi)
    {
        PointBufferPtr buf = *pi;
        cnt += buf->size();
    }
    return cnt;
}


MetadataNode PipelineManager::getMetadata() const
{
    MetadataNode output("stages");

    for (auto si = m_stages.begin(); si != m_stages.end(); ++si)
        output.add((*si)->getMetadata());

    return output;
}

} // namespace pdal
