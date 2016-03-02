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

#include "PipelineReaderJSON.hpp"

#include <pdal/Filter.hpp>
#include <pdal/PipelineManager.hpp>
#include <pdal/Options.hpp>
#include <pdal/util/FileUtils.hpp>
#include <pdal/util/Utils.hpp>

#include <json/json.h>
#include <json/json-forwards.h>

#include <memory>
#include <vector>

#ifndef _WIN32
#include <wordexp.h>
#endif

namespace pdal
{

// ------------------------------------------------------------------------

// this class helps keep tracks of what child nodes we've seen, so we
// can keep all the error checking in one place
class PipelineReaderJSON::StageParserContext
{
public:
    enum Cardinality { None, One, Many };

    StageParserContext()
        : m_numTypes(0)
        , m_cardinality(One)
        , m_numStages(0)
    {}

    void setCardinality(Cardinality cardinality)
    {
        m_cardinality = cardinality;
    }

    void addType()
    {
        ++m_numTypes;
    }

    int getNumTypes()
    {
        return m_numTypes;
    }

    void addStage()
    {
        ++m_numStages;
    }

    void addUnknown(const std::string& name)
    {
        throw pdal_error("unknown child of element: " + name);
    }

    void validate()
    {
        if (m_numTypes == 0)
            throw pdal_error("PipelineReaderJSON: expected Type element missing");
        if (m_numTypes > 1)
            throw pdal_error("PipelineReaderJSON: extra Type element found");

        if (m_cardinality == None)
        {
            if (m_numStages != 0)
                throw pdal_error("PipelineReaderJSON: found child stages where "
                                 "none were expected");
        }
        if (m_cardinality == One)
        {
            if (m_numStages == 0)
                throw pdal_error("PipelineReaderJSON: "
                                 "expected child stage missing");
            if (m_numStages > 1)
                throw pdal_error("PipelineReaderJSON: extra child stages found");
        }
        if (m_cardinality == Many)
        {
            if (m_numStages == 0)
                throw pdal_error("PipelineReaderJSON: expected child stage "
                                 "missing");
        }
    }

private:
    int m_numTypes;
    Cardinality m_cardinality; // num child stages allowed
    int m_numStages;
};


PipelineReaderJSON::PipelineReaderJSON(PipelineManager& manager, bool isDebug,
                                       uint32_t verboseLevel) :
    m_manager(manager) , m_isDebug(isDebug) , m_verboseLevel(verboseLevel)
{
    if (m_isDebug)
    {
        Option opt("debug", true);
        m_baseOptions.add(opt);
    }
    if (m_verboseLevel)
    {
        Option opt("verbose", m_verboseLevel);
        m_baseOptions.add(opt);
    }
}


Stage *PipelineReaderJSON::parseReaderByFilename(const std::string& filename)
{
    Options options;

    StageParserContext context;
    std::string type;

    try
    {
        type = StageFactory::inferReaderDriver(filename);
        if (!type.empty())
        {
            context.addType();
        }

        Option opt("filename", filename);
        options += opt;
    }
    catch (Option::not_found)
    {}

    context.setCardinality(StageParserContext::None);
    context.validate();

    Stage& reader(m_manager.addReader(type));
    reader.setOptions(options);

    return &reader;
}


Stage *PipelineReaderJSON::parseWriterByFilename(const std::string& filename)
{
    Options options;

    StageFactory f;
    StageParserContext context;
    std::string type;

    try
    {
        type = f.inferWriterDriver(filename);
        if (type.empty())
            throw pdal_error("Cannot determine output file type of " +
                             filename);

        options += f.inferWriterOptionsChanges(filename);
        context.addType();
    }
    catch (Option::not_found)
    {}

    context.setCardinality(StageParserContext::None);
    context.validate();

    Stage& writer(m_manager.addWriter(type));
    writer.setOptions(options);

    return &writer;
}


void PipelineReaderJSON::parseElement_Pipeline(const Json::Value& tree)
{
    StageFactory f;
    std::map<std::string, Stage*> tags;
    std::vector<Stage*> stages;
    std::vector<Stage*> firstReaders;
    bool onlyReaders = true;
    bool firstNonReader = true;

    size_t i = 0;
    for (auto const& node : tree)
    {
        std::vector<std::string> inputs;
        Stage* stage = NULL;

        bool curStageIsReader = false;
        bool filenameIsSet = false;

        // strings are assumed to be filenames
        if (node.isString())
        {
            std::string filename = node.asString();
            // surely not common, but if there is only one string, it must be a
            // reader and not a writer
            if (tree.size() == 1)
            {
                stage = parseReaderByFilename(filename);
                curStageIsReader = true;
                filenameIsSet = true;
            }
            // all filenames assumed to be readers...
            else if (i < tree.size()-1)
            {
                stage = parseReaderByFilename(filename);
                if (onlyReaders)
                    firstReaders.push_back(stage);
                curStageIsReader = true;
                filenameIsSet = true;
            }
            // ...except the last
            else
            {
                stage = parseWriterByFilename(filename);
                onlyReaders = false;
                filenameIsSet = true;
            }
        }
        else
        {
            std::string type, filename, tag;
            if (node.isMember("type"))
                type = node["type"].asString();
            if (node.isMember("filename"))
                filename = node["filename"].asString();
            if (node.isMember("tag"))
                tag = node["tag"].asString();
            if (node.isMember("inputs"))
            {
                for (auto const& input : node["inputs"])
                {
                    if (input.isString())
                        inputs.push_back(input.asString());
                    else
                        throw pdal_error("Stage inputs must be specified as "
                            "a string");
                }
            }

            if (!type.empty())
            {
                if (Utils::startsWith(type, "readers."))
                {
                    stage = &m_manager.addReader(type);
                    if (onlyReaders)
                        firstReaders.push_back(stage);
                    curStageIsReader = true;
                }
                else if (Utils::startsWith(type, "filters."))
                {
                    stage = &m_manager.addFilter(type);
                    onlyReaders = false;
                }
                else if (Utils::startsWith(type, "writers."))
                {
                    stage = &m_manager.addWriter(type);
                    onlyReaders = false;
                }
                else
                    throw pdal_error("Could not determine type of " + type);
            }
            else if (!filename.empty())
            {
                if (i < tree.size()-1)
                {
                    stage = parseReaderByFilename(filename);
                    if (onlyReaders)
                        firstReaders.push_back(stage);
                    curStageIsReader = true;
                    filenameIsSet = true;
                }
                else
                {
                    stage = parseWriterByFilename(filename);
                    filenameIsSet = true;
                    onlyReaders = false;
                }
            }

            if (!tag.empty())
            {
                if (tags[tag])
                    throw pdal_error("Duplicate tag " + tag);

                tags[tag] = stage;
            }

            Options options(m_baseOptions);
            for (auto const& name : node.getMemberNames())
            {
                if (name.compare("type") == 0)
                    continue;
                if (name.compare("inputs") == 0)
                    continue;
                if (name.compare("tag") == 0)
                    continue;
                if (filenameIsSet && name.compare("filename") == 0)
                    continue;

                Option opt(name, node[name].asString());
                options.add(opt);
            }

            stage->addOptions(options);
        }

        if (!inputs.empty())
        {
            for (auto const& input : inputs)
            {
                if (!tags[input])
                    throw pdal_error("Invalid pipeline, undefined stage " + input);

                stage->setInput(*tags[input]);
            }
        }
        else
        {
            if (i && !curStageIsReader)
            {
                if (firstReaders.size() > 0 && firstNonReader)
                {
                    firstNonReader = false;
                    for (Stage* s : firstReaders)
                        stage->setInput(*s);
                }
                else
                {
                    stage->setInput(*stages[i-1]);
                }
            }
        }

        stages.push_back(stage);

        i++;
    }
}


void PipelineReaderJSON::readPipeline(std::istream& input)
{
    Json::Value root;
    Json::Reader jsonReader;
    if (!jsonReader.parse(input, root))
        throw pdal_error("PipelineReaderJSON: unable to parse pipeline");

    Json::Value subtree = root["pipeline"];
    if (!subtree)
        throw pdal_error("PipelineReaderJSON: root element is not a Pipeline");
    parseElement_Pipeline(subtree);
}


void PipelineReaderJSON::readPipeline(const std::string& filename)
{
    m_inputJSONFile = filename;

    std::istream* input = FileUtils::openFile(filename);
    if (!input)
    {
        std::ostringstream oss;
        oss << "Unable to open stream for file '" << filename << "'";
        throw pdal_error(oss.str());
    }

    try
    {
        readPipeline(*input);
    }
    catch (const pdal_error& error)
    {
        throw error;
    }
    catch (...)
    {
        FileUtils::closeFile(input);
        std::ostringstream oss;
        oss << "Unable to process pipeline file \"" << filename << "\"." <<
            "  JSON is invalid.";
        throw pdal_error(oss.str());
    }

    FileUtils::closeFile(input);

    m_inputJSONFile = "";
}


} // namespace pdal
