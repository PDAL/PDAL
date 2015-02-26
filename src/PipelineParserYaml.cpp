/******************************************************************************
* Copyright (c) 2015, Peter J. Gadomski (pete.gadomski@gmail.com)
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

#include "PipelineParserYaml.hpp"

#include <yaml-cpp/yaml.h>


namespace pdal
{


PipelineParserYaml::PipelineParserYaml(PipelineManager& manager, const Options& baseOptions)
    : PipelineParser(manager, baseOptions)
{}


bool PipelineParserYaml::parse(std::istream& stream)
{
    YAML::Node pipeline = YAML::Load(stream);
    return parseYamlNode(pipeline);
}


bool PipelineParserYaml::parse(const std::string& filename)
{
    YAML::Node pipeline = YAML::LoadFile(filename);
    return parseYamlNode(pipeline);
}

bool PipelineParserYaml::parseYamlNode(YAML::Node& pipeline)
{
    // We do some iteration backflips becuase yaml-cpp Nodes don't support
    // the back() operator or iterator arithmatic (end() - 1)
    std::size_t idx = 0;
    auto it = pipeline.begin();
    std::string readerName = it->first.as<std::string>();
    Stage* lastStage = addReader(readerName);
    addOptions(lastStage, it->second);

    if (pipeline.size() == 1)
    {
        return false;
    }

    while ((++idx) < pipeline.size() - 1)
    {
        std::string filterName = (++it)->first.as<std::string>();
        lastStage = addFilter(filterName, lastStage);
        addOptions(lastStage, it->second);
    }

    bool lastStageIsWriter = true;
    std::string writerName = (++it)->first.as<std::string>();
    try
    {
        lastStage = addWriter(writerName, lastStage);
    }
    catch (pdal_error& e)
    {
        try
        {
            lastStage = addFilter(writerName, lastStage);
            lastStageIsWriter = false;
        }
        catch (pdal_error& e2)
        {
            throw e;
        }
    }
    addOptions(lastStage, it->second);
    return lastStageIsWriter;
}

void PipelineParserYaml::addOptions(Stage* stage, YAML::Node& node)
{
    Options options(getBaseOptions());
    for (auto it = node.begin(); it != node.end(); ++it)
    {
        options.add(it->first.as<std::string>(), it->second.as<std::string>());
    }
    stage->setOptions(options);
}


}
