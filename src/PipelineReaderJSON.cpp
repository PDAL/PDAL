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

    void validate()
    {
        if (m_numTypes == 0)
            throw pdal_error("JSON pipeline: Expected 'type' element missing");
        if (m_numTypes > 1)
            throw pdal_error("JSON pipeline: Extra Type element found");

        if (m_cardinality == None)
        {
            if (m_numStages != 0)
                throw pdal_error("JSON pipeline: Found child stages where "
                    "none were expected");
        }
        if (m_cardinality == One)
        {
            if (m_numStages == 0)
                throw pdal_error("JSON pipeline: Expected child stage missing");
            if (m_numStages > 1)
                throw pdal_error("JSON pipeline: Extra child stages found");
        }
        if (m_cardinality == Many)
        {
            if (m_numStages == 0)
                throw pdal_error("JSON pipeline: Expected child stage missing");
        }
    }

private:
    int m_numTypes;
    Cardinality m_cardinality; // num child stages allowed
    int m_numStages;
};


PipelineReaderJSON::PipelineReaderJSON(PipelineManager& manager) :
    m_manager(manager)
{}


Stage *PipelineReaderJSON::parseReaderByFilename(const std::string& filename)
{
    StageParserContext context;

    std::string type = StageFactory::inferReaderDriver(filename);
    if (!type.empty())
        context.addType();
    context.setCardinality(StageParserContext::None);
    context.validate();

    Stage& reader = m_manager.addReader(type);

    Options ops;
    ops.add("filename", filename);
    reader.setOptions(ops);

    return &reader;
}


Stage *PipelineReaderJSON::parseWriterByFilename(const std::string& filename)
{
    StageParserContext context;

    std::string type = StageFactory::inferWriterDriver(filename);
    if (type.empty())
        throw pdal_error("PipelineReaderJSON: "
                "Cannot determine output file type of " +
                filename);

    Options options;
    options.add("filename", filename);
    options += StageFactory::inferWriterOptionsChanges(filename);

    context.addType();
    context.setCardinality(StageParserContext::None);
    context.validate();

    Stage& writer(m_manager.addWriter(type));
    writer.addOptions(options);
    return &writer;
}


void PipelineReaderJSON::parseElement_Pipeline(const Json::Value& tree)
{
    std::map<std::string, Stage*> tags;
    std::vector<Stage*> stages;
    std::vector<Stage*> firstReaders;
    bool onlyReaders = true;
    bool firstNonReader = true;

    size_t i = 0;
    for (auto const& node : tree)
    {
        Stage* stage = NULL;
        StringList sInputs;

        bool curStageIsReader = false;

        // strings are assumed to be filenames
        if (node.isString())
        {
            std::string filename = node.asString();
            // surely not common, but if there is only one string, it must be a
            // reader and not a writer
            // all filenames assumed to be readers except the last.
            if (tree.size() == 1 || (i < tree.size()-1))
            {
                stage = parseReaderByFilename(filename);
                if (onlyReaders)
                    firstReaders.push_back(stage);
                curStageIsReader = true;
            }
            else
            {
                stage = parseWriterByFilename(filename);
                onlyReaders = false;
            }
        }
        else
        {
            Json::Value type, filename, tag, inputs;
            if (node.isMember("type"))
                type = node["type"];
            if (node.isMember("filename"))
                filename = node["filename"];
            if (node.isMember("tag"))
                tag = node["tag"];
            if (node.isMember("inputs"))
                inputs = node["inputs"];

            if (!inputs.isNull())
            {
                for (auto const& input : node["inputs"])
                {
                    if (input.isString())
                        sInputs.push_back(input.asString());
                    else
                        throw pdal_error("JSON pipeline: "
                            "Stage inputs must be specified as a string");
                }
            }

            std::string sFilename;
            if (!filename.isNull())
            {
                if (!filename.isString())
                    throw pdal_error("JSON pipeline: filenames must "
                        "be specified as a string");
                sFilename = filename.asString();
            }

            stage = NULL;
            if (!type.isNull())
            {
                std::string sType;
                if (type.isString())
                    sType = type.asString();

                // PipelineManager will throw if we can't create
                // the requested stage type.
                if (Utils::startsWith(sType, "readers."))
                {
                    stage = &m_manager.addReader(sType);
                    if (onlyReaders)
                        firstReaders.push_back(stage);
                    curStageIsReader = true;
                }
                else if (Utils::startsWith(sType, "filters."))
                {
                    stage = &m_manager.addFilter(sType);
                    onlyReaders = false;
                }
                else if (Utils::startsWith(sType, "writers."))
                {
                    stage = &m_manager.addWriter(sType);
                    onlyReaders = false;
                }
                else
                    throw pdal_error("JSON pipeline: could not create stage "
                        "for type " + sType);
            }

            if (!stage && !sFilename.empty())
            {
                // Anything other than the last entry in the tree is
                // a reader.
                if (i < tree.size() - 1)
                {
                    stage = parseReaderByFilename(sFilename);
                    if (onlyReaders)
                        firstReaders.push_back(stage);
                    curStageIsReader = true;
                }
                // If the last entry is a filename, then it's writer.
                else
                {
                    stage = parseWriterByFilename(sFilename);
                    onlyReaders = false;
                }
            }

            if (!stage)
                throw pdal_error("JSON pipeline: Found neither a stage type "
                    "nor a filename with which a stage type could be "
                    "inferred.");

            if (!tag.isNull())
            {
                std::string sTag;
                if (!tag.isString())
                    throw pdal_error("JSON pipeline: tags must "
                        "be specified as a string");
                sTag = tag.asString();
                if (tags.find(sTag) != tags.end())
                    throw pdal_error("JSON pipeline: duplicate tag " + sTag);
                tags[sTag] = stage;
            }

            Options options;
            for (auto const& name : node.getMemberNames())
            {
                if (name.compare("type") == 0)
                    continue;
                if (name.compare("inputs") == 0)
                    continue;
                if (name.compare("tag") == 0)
                    continue;

                if (!node[name].isConvertibleTo(Json::stringValue))
                    throw pdal_error("JSON pipeline: Value of stage option '" +
                        name + "' must be convertible to a string.");
                options.add(name, node[name].asString());
            }

            m_manager.setOptions(*stage, options);
        }

        if (!sInputs.empty())
        {
            for (auto const& input : sInputs)
            {
                auto ti = tags.find(input);
                if (ti == tags.end())
                    throw pdal_error("JSON pipeline: Invalid pipeline, "
                        "undefined stage '" + input + "'");
                stage->setInput(*(ti->second));
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
        throw pdal_error("JSON pipeline: Unable to parse pipeline");

    Json::Value subtree = root["pipeline"];
    if (!subtree)
        throw pdal_error("JSON pipeline: Root element is not a Pipeline");
    parseElement_Pipeline(subtree);
}


void PipelineReaderJSON::readPipeline(const std::string& filename)
{
    m_inputJSONFile = filename;

    std::istream* input = FileUtils::openFile(filename);
    if (!input)
    {
        throw pdal_error("JSON pipeline: Unable to open stream for "
            "file \"" + filename + "\"");
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
        throw pdal_error("JSON pipeline: Unable to process pipeline "
            "file \""+ filename + "\". JSON is invalid.");
    }

    FileUtils::closeFile(input);

    m_inputJSONFile = "";
}

} // namespace pdal
