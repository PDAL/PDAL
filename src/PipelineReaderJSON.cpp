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
#include <pdal/PluginManager.hpp>
#include <pdal/Options.hpp>
#include <pdal/PDALUtils.hpp>
#include <pdal/util/Utils.hpp>

#include <json/json.h>

#include <memory>
#include <vector>

namespace pdal
{

PipelineReaderJSON::PipelineReaderJSON(PipelineManager& manager) :
    m_manager(manager)
{}


void PipelineReaderJSON::parsePipeline(Json::Value& tree)
{
    std::map<std::string, Stage*> tags;
    std::vector<Stage*> inputs;

    Json::ArrayIndex last = tree.size() - 1;
    for (Json::ArrayIndex i = 0; i < tree.size(); ++i)
    {
        Json::Value& node = tree[i];

        std::string filename;
        std::string tag;
        std::string type;
        std::vector<Stage*> specifiedInputs;
        Options options;

        // strings are assumed to be filenames
        if (node.isString())
        {
            filename = node.asString();
        }
        else
        {
            type = extractType(node);
            filename = extractFilename(node);
            tag = extractTag(node, tags);
            specifiedInputs = extractInputs(node, tags);
            if (!specifiedInputs.empty())
                inputs = specifiedInputs;
            options = extractOptions(node);
        }

        Stage *s = nullptr;

        // The type is inferred from a filename as a reader if it's not
        // the last stage or if there's only one.
        if ((type.empty() && (i == 0 || i != last)) ||
            Utils::startsWith(type, "readers."))
        {
            s = &m_manager.makeReader(filename, type, options);
            if (specifiedInputs.size())
                throw pdal_error("JSON pipeline: Inputs not permitted for "
                    " reader: '" + filename + "'.");
            inputs.push_back(s);
        }
        else if (type.empty() || Utils::startsWith(type, "writers."))
        {
            s = &m_manager.makeWriter(filename, type, options);
            for (Stage *ts : inputs)
                s->setInput(*ts);
            inputs.clear();
        }
        else
        {
            if (filename.size())
                options.add("filename", filename);
            s = &m_manager.makeFilter(type, options);
            for (Stage *ts : inputs)
                s->setInput(*ts);
            inputs.clear();
            inputs.push_back(s);
        }
        // s should be valid at this point.  makeXXX will throw if the stage
        // couldn't be constructed.
        if (tag.size())
            tags[tag] = s;
    }
}


void PipelineReaderJSON::readPipeline(std::istream& input)
{
    Json::Value root;
    Json::Reader jsonReader;
    if (!jsonReader.parse(input, root))
        throw pdal_error("JSON pipeline: Unable to parse pipeline");

    Json::Value& subtree = root["pipeline"];
    if (!subtree)
        throw pdal_error("JSON pipeline: Root element is not a Pipeline");
    parsePipeline(subtree);
}


void PipelineReaderJSON::readPipeline(const std::string& filename)
{
    m_inputJSONFile = filename;

    std::istream* input = Utils::openFile(filename);
    if (!input)
    {
        throw pdal_error("JSON pipeline: Unable to open stream for "
            "file \"" + filename + "\"");
    }

    try
    {
        readPipeline(*input);
    }
    catch (...)
    {
        Utils::closeFile(input);
        throw;
    }

    Utils::closeFile(input);
    m_inputJSONFile = "";
}


std::string PipelineReaderJSON::extractType(Json::Value& node)
{
    std::string type;

    if (node.isMember("type"))
    {
        Json::Value& val = node["type"];
        if (!val.isNull())
        {
            if (val.isString())
                type = val.asString();
            else
                throw pdal_error("JSON pipeline: 'type' must be specified as "
                        "a string.");
        }
        node.removeMember("type");
        if (node.isMember("type"))
            throw pdal_error("JSON pipeline: found duplicate 'type' "
               "entry in stage definition.");
    }
    return type;
}


std::string PipelineReaderJSON::extractFilename(Json::Value& node)
{
    std::string filename;

    if (node.isMember("filename"))
    {
        Json::Value& val = node["filename"];
        if (!val.isNull())
        {
            if (val.isString())
                filename = val.asString();
            else
                throw pdal_error("JSON pipeline: 'filename' must be "
                    "specified as a string.");
        }
        node.removeMember("filename");
        if (node.isMember("filename"))
            throw pdal_error("JSON pipeline: found duplicate 'filename' "
               "entry in stage definition.");
    }
    return filename;
}


std::string PipelineReaderJSON::extractTag(Json::Value& node, TagMap& tags)
{
    std::string tag;

    if (node.isMember("tag"))
    {
        Json::Value& val = node["tag"];
        if (!val.isNull())
        {
            if (val.isString())
            {
                tag = val.asString();
                if (tags.find(tag) != tags.end())
                    throw pdal_error("JSON pipeline: duplicate tag '" +
                        tag + "'.");
            }
            else
                throw pdal_error("JSON pipeline: 'tag' must be "
                    "specified as a string.");
        }
        node.removeMember("tag");
        if (node.isMember("tag"))
            throw pdal_error("JSON pipeline: found duplicate 'tag' "
               "entry in stage definition.");
    }
    return tag;
}


std::vector<Stage *> PipelineReaderJSON::extractInputs(Json::Value& node,
    TagMap& tags)
{
    std::vector<Stage *> inputs;
    std::string filename;

    if (node.isMember("inputs"))
    {
        Json::Value& val = node["inputs"];
        if (!val.isNull())
        {
            for (const Json::Value& input : node["inputs"])
            {
                if (input.isString())
                {
                    std::string tag = input.asString();
                    auto ii = tags.find(tag);
                    if (ii == tags.end())
                        throw pdal_error("JSON pipeline: Invalid pipeline: "
                            "undefined stage tag '" + tag + "'.");
                    else
                        inputs.push_back(ii->second);
                }
                else
                    throw pdal_error("JSON pipeline: 'inputs' tag must "
                        " be specified as a string.");
            }
        }
        node.removeMember("inputs");
        if (node.isMember("inputs"))
            throw pdal_error("JSON pipeline: found duplicate 'inputs' "
               "entry in stage definition.");
    }
    return inputs;
}


Options PipelineReaderJSON::extractOptions(Json::Value& node)
{
    Options options;

    for (const std::string& name : node.getMemberNames())
    {
        if (name == "plugin")
        {
            PluginManager::loadPlugin(node[name].asString());

            // Don't actually put a "plugin" option on
            // any stage
            continue;
        }

        if (node[name].isString())
            options.add(name, node[name].asString());
        else if (node[name].isInt())
            options.add(name, node[name].asInt64());
        else if (node[name].isUInt())
            options.add(name, node[name].asUInt64());
        else if (node[name].isDouble())
            options.add(name, node[name].asDouble());
        else if (node[name].isBool())
            options.add(name, node[name].asBool());
        else if (node[name].isNull())
            options.add(name, "");
        else
            throw pdal_error("JSON pipeline: Value of stage option '" +
                name + "' cannot be converted.");
    }
    node.clear();
    return options;
}

} // namespace pdal
