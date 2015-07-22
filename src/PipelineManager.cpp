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

namespace pdal
{

Stage& PipelineManager::addReader(const std::string& type)
{
    Stage *r = m_factory.createStage(type);
    if (!r)
    {
        std::ostringstream ss;
        ss << "Couldn't create reader stage of type '" << type << "'.";
        throw pdal_error(ss.str());
    }
    r->setProgressFd(m_progressFd);
    m_stages.push_back(std::unique_ptr<Stage>(r));
    return *r;
}


Stage& PipelineManager::addFilter(const std::string& type)
{
    Stage *stage = m_factory.createStage(type);
    if (!stage)
    {
        std::ostringstream ss;
        ss << "Couldn't create filter stage of type '" << type << "'.";
        throw pdal_error(ss.str());
    }
    stage->setProgressFd(m_progressFd);
    m_stages.push_back(std::unique_ptr<Stage>(stage));
    return *stage;
}


Stage& PipelineManager::addWriter(const std::string& type)
{
    Stage *writer = m_factory.createStage(type);
    if (!writer)
    {
        std::ostringstream ss;
        ss << "Couldn't create writer stage of type '" << type << "'.";
        throw pdal_error(ss.str());
    }
    writer->setProgressFd(m_progressFd);
    m_stages.push_back(std::unique_ptr<Stage>(writer));
    return *writer;
}


void PipelineManager::prepare() const
{
    Stage *s = getStage();
    if (s)
       s->prepare(m_table);
}


point_count_t PipelineManager::execute()
{
    prepare();

    Stage *s = getStage();
    if (!s)
        return 0;
    m_viewSet = s->execute(m_table);
    point_count_t cnt = 0;
    for (auto pi = m_viewSet.begin(); pi != m_viewSet.end(); ++pi)
    {
        PointViewPtr view = *pi;
        cnt += view->size();
    }
    return cnt;
}


MetadataNode PipelineManager::getMetadata() const
{
    MetadataNode output("stages");

    for (auto si = m_stages.begin(); si != m_stages.end(); ++si)
    {
        Stage *s = si->get();
        output.add(s->getMetadata());
    }
    return output;
}

} // namespace pdal
