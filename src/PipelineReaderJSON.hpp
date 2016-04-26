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

#pragma once

#include <json/json-forwards.h>

#include <vector>
#include <string>

#include <pdal/Options.hpp>
#include <pdal/StageFactory.hpp>

namespace pdal
{

class Stage;
class PipelineManager;

class PDAL_DLL PipelineReaderJSON
{
    friend class PipelineManager;

private:
    typedef std::map<std::string, Stage *> TagMap;

    PipelineReaderJSON(PipelineManager&);
    void readPipeline(const std::string& filename);
    void readPipeline(std::istream& input);
    void parsePipeline(Json::Value&);
    std::string extractType(Json::Value& node);
    std::string extractFilename(Json::Value& node);
    std::string extractTag(Json::Value& node, TagMap& tags);
    std::vector<Stage *> extractInputs(Json::Value& node, TagMap& tags);
    Options extractOptions(Json::Value& node);

    PipelineManager& m_manager;
    std::string m_inputJSONFile;

    PipelineReaderJSON& operator=(const PipelineReaderJSON&); // not implemented
    PipelineReaderJSON(const PipelineReaderJSON&); // not implemented
};

} // namespace pdal
