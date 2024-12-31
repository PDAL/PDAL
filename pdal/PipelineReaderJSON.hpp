/****************************************************************************** * Copyright (c) 2011, Michael P. Gerlek (mpg@flaxen.com)
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

#include <pdal/pdal_internal.hpp>
#include <pdal/JsonFwd.hpp>
#include <pdal/StageFactory.hpp>

#include <vector>
#include <string>

#include <pdal/FileSpec.hpp>
#include <pdal/Options.hpp>
#include <pdal/StageFactory.hpp>

namespace pdal
{

class Stage;
class PipelineManager;

class PDAL_EXPORT PipelineReaderJSON
{
    friend class PipelineManager;

public:
    PipelineReaderJSON(PipelineManager&);

private:
    PipelineReaderJSON& operator=(const PipelineReaderJSON&) = delete;
    PipelineReaderJSON(const PipelineReaderJSON&) = delete;

    typedef std::map<std::string, Stage *> TagMap;

    void parsePipeline(NL::json&);
    void readPipeline(const std::string& filename);
    void readPipeline(std::istream& input);
    std::string extractType(NL::json& node);
    FileSpec extractFilename(NL::json& node);
    void extractPath(NL::json& node, FileSpec& spec);
    void extractHeaders(NL::json& node, FileSpec& spec);
    void extractQuery(NL::json& node, FileSpec& spec);
    StringList extractStringList(const std::string& name, NL::json& node);
    std::string extractTag(NL::json& node, TagMap& tags);
    std::vector<Stage *> extractInputs(NL::json& node, TagMap& tags);
    Options extractOptions(NL::json& node);
    void handleInputTag(const std::string& tag, const TagMap& tags,
        std::vector<Stage *>& inputs);

    PipelineManager& m_manager;
};

} // namespace pdal
