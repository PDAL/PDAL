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

#ifndef INCLUDED_PIPELINEMANAGER_HPP
#define INCLUDED_PIPELINEMANAGER_HPP

#include <vector>

#include <pdal/Stage.hpp>
#include <pdal/Filter.hpp>
#include <pdal/Writer.hpp>


namespace pdal
{

class OptionsNew;

class PDAL_DLL PipelineManager
{
public:
    PipelineManager();

    boost::uint32_t addReader(const std::string& type, const OptionsNew&);
    boost::uint32_t addFilter(const std::string& type, boost::uint32_t prevStage, const OptionsNew&);
    boost::uint32_t addFilter(const std::string& type, const std::vector<boost::uint32_t>& prevStages, const OptionsNew&);
    boost::uint32_t addWriter(const std::string& type, boost::uint32_t prevStage, const OptionsNew&);
    
    Stage* getStage(boost::uint32_t);
    Filter* getFilter(boost::uint32_t);
    Writer* getWriter(boost::uint32_t);

    typedef Stage* readerCreatorFunction(const OptionsNew&);
    typedef Filter* filter1CreatorFunction(boost::uint32_t prevStage, const OptionsNew&);
    typedef Filter* filterNCreatorFunction(const std::vector<boost::uint32_t>& prevStage, const OptionsNew&);
    typedef Writer* writerCreatorFunction(boost::uint32_t prevStage, const OptionsNew&);
    void registerReader(const std::string& type, readerCreatorFunction);
    void registerFilter(const std::string& type, filter1CreatorFunction);
    void registerFilter(const std::string& type, filterNCreatorFunction);
    void registerWriter(const std::string& type, writerCreatorFunction);

private:
    void registerKnownReaders();
    void registerKnownFilters();
    void registerKnownWriters();

    PipelineManager& operator=(const PipelineManager&); // not implemented
    PipelineManager(const PipelineManager&); // not implemented
};

} // namespace pdal

#endif
