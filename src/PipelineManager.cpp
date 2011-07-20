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


PipelineManager::PipelineManager()
{
}

/*

    boost::uint32_t PipelineManager::addReader(const std::string& type, const OptionsNew&);
    boost::uint32_t PipelineManager::addFilter(const std::string& type, boost::uint32_t prevStage, const OptionsNew&);
    boost::uint32_t PipelineManager::addFilter(const std::string& type, const std::vector<boost::uint32_t>& prevStages, const OptionsNew&);
    boost::uint32_t PipelineManager::addWriter(const std::string& type, boost::uint32_t prevStage, const OptionsNew&);
    
    Stage* PipelineManager::getStage(boost::uint32_t);
    Filter* PipelineManager::getFilter(boost::uint32_t);
    Writer* PipelineManager::getWriter(boost::uint32_t);

    typedef Stage* PipelineManager::readerCreatorFunction(const OptionsNew&);
    typedef Filter* PipelineManager::filter1CreatorFunction(boost::uint32_t prevStage, const OptionsNew&);
    typedef Filter* PipelineManager::filterNCreatorFunction(const std::vector<boost::uint32_t>& prevStage, const OptionsNew&);
    typedef Writer* PipelineManager::writerCreatorFunction(boost::uint32_t prevStage, const OptionsNew&);
    void PipelineManager::registerReader(const std::string& type, readerCreatorFunction);
    void PipelineManager::registerFilter(const std::string& type, filter1CreatorFunction);
    void PipelineManager::registerFilter(const std::string& type, filterNCreatorFunction);
    void PipelineManager::registerWriter(const std::string& type, writCreatorFunction);

    void PipelineManager::registerKnownReaders();
    void PipelineManager::registerKnownFilters();
    void PipelineManager::registerKnownWriters();
    */

} // namespace pdal
