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

#include <pdal/pdal.hpp>
#include <pdal/StageFactory.hpp>

#include <boost/shared_ptr.hpp>

#include <libxml/xmlmemory.h>
#include <libxml/parser.h>

#include <vector>
#include <map>
#include <string>


namespace pdal
{

class Options;


class PDAL_DLL PipelineManager
{
public:
    PipelineManager();
    ~PipelineManager();

    ReaderPtr addReader(const std::string& type, const Options&);
    FilterPtr addFilter(const std::string& type, const DataStagePtr& prevStage, const Options&);
    MultiFilterPtr addMultiFilter(const std::string& type, const std::vector<const DataStagePtr>& prevStages, const Options&);
    WriterPtr addWriter(const std::string& type, const DataStagePtr& prevStage, const Options&);
    
    void readXml(const std::string&);

private:
    StagePtr parsePipeline(xmlDocPtr doc, xmlNodePtr cur);
    StagePtr parseStage(xmlDocPtr doc, xmlNodePtr cur);
    DataStagePtr parseDataStage(xmlDocPtr doc, xmlNodePtr cur);
    ReaderPtr parseReader(xmlDocPtr doc, xmlNodePtr cur);
    FilterPtr parseFilter(xmlDocPtr doc, xmlNodePtr cur);
    MultiFilterPtr parseMultiFilter(xmlDocPtr doc, xmlNodePtr cur);
    WriterPtr parseWriter(xmlDocPtr doc, xmlNodePtr cur);

    void parseOption(xmlDocPtr doc, xmlNodePtr cur, Options&);
    Options parseOptions(xmlDocPtr doc, xmlNodePtr cur);

    StageFactory m_factory;

    typedef std::vector<StagePtr> StagePtrList;
    StagePtrList m_stages;

    PipelineManager& operator=(const PipelineManager&); // not implemented
    PipelineManager(const PipelineManager&); // not implemented
};


} // namespace pdal

#endif
