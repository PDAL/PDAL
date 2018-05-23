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


#include <pdal/Filter.hpp>
#include <pdal/PipelineReaderJSON.hpp>
#include <pdal/PipelineManager.hpp>
#include <pdal/PluginManager.hpp>
#include <pdal/Options.hpp>
#include <pdal/util/FileUtils.hpp>
#include <pdal/util/Algorithm.hpp>
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
    TagMap tags;
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
            StringList files = FileUtils::glob(filename);
            if (files.empty())
                files.push_back(filename);

            for (const std::string& path : files)
            {
                StageCreationOptions ops { path, type, nullptr, options, tag };
                s = &m_manager.makeReader(ops);

                if (specifiedInputs.size())
                    throw pdal_error("JSON pipeline: Inputs not permitted for "
                        " reader: '" + path + "'.");
                inputs.push_back(s);
            }
        }
        else if (type.empty() || Utils::startsWith(type, "writers."))
        {
            StageCreationOptions ops { filename, type, nullptr, options, tag };
            s = &m_manager.makeWriter(ops);
            for (Stage *ts : inputs)
                s->setInput(*ts);
            inputs.clear();
        }
        else
        {
            if (filename.size())
                options.add("filename", filename);
            StageCreationOptions ops { "", type, nullptr, options, tag };
            s = &m_manager.makeFilter(ops);
            for (Stage *ts : inputs)
                s->setInput(*ts);
            inputs.clear();
            inputs.push_back(s);
        }
        // 's' should be valid at this point.  makeXXX will throw if the stage
        // couldn't be constructed.
        if (tag.size())
            tags[tag] = s;
    }
}


void PipelineReaderJSON::readPipeline(std::istream& input)
{
    Json::Value root;
    Json::CharReaderBuilder builder;
    builder["rejectDupKeys"] = true;
    std::string err;
    if (!parseFromStream(builder, input, &root, &err))
    {
        err = "JSON pipeline: Unable to parse pipeline:\n" + err;
        throw pdal_error(err);
    }

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
                throw pdal_error("JSON pipeline: tag must be "
                    "specified as a string.");
        }
        node.removeMember("tag");
        if (node.isMember("tag"))
            throw pdal_error("JSON pipeline: found duplicate 'tag' "
               "entry in stage definition.");
        std::string::size_type pos = 0;
        if (!Stage::parseTagName(tag, pos) || pos != tag.size())
            throw pdal_error("JSON pipeline: Invalid tag name '" + tag + "'.  "
                "Must start with letter.  Remainder can be letters, "
                "digits or underscores.");
    }
    return tag;
}


void PipelineReaderJSON::handleInputTag(const std::string& tag,
    const TagMap& tags, std::vector<Stage *>& inputs)
{
    auto ii = tags.find(tag);
    if (ii == tags.end())
        throw pdal_error("JSON pipeline: Invalid pipeline: "
            "undefined stage tag '" + tag + "'.");
    else
        inputs.push_back(ii->second);
}


std::vector<Stage *> PipelineReaderJSON::extractInputs(Json::Value& node,
    TagMap& tags)
{
    std::vector<Stage *> inputs;
    std::string filename;

    if (node.isMember("inputs"))
    {
        Json::Value& val = node["inputs"];
        if (val.isString())
            handleInputTag(val.asString(), tags, inputs);
        else if (val.isArray())
        {
            for (const Json::Value& input : node["inputs"])
            {
                if (!input.isString())
                    throw pdal_error("JSON pipeline: 'inputs' tag must "
                            " be specified as a string or array of strings.");
                handleInputTag(input.asString(), tags, inputs);
            }
        }
        else
            throw pdal_error("JSON pipeline: 'inputs' tag must "
                    " be specified as a string or array of strings.");
        node.removeMember("inputs");
        if (node.isMember("inputs"))
            throw pdal_error("JSON pipeline: found duplicate 'inputs' "
               "entry in stage definition.");
    }
    return inputs;
}

namespace
{

bool extractOption(Options& options, const std::string& name,
    const Json::Value& node)
{
    if (node.isString())
        options.add(name, node.asString());
    else if (node.isInt())
        options.add(name, node.asInt64());
    else if (node.isUInt())
        options.add(name, node.asUInt64());
    else if (node.isDouble())
        options.add(name, node.asDouble());
    else if (node.isBool())
        options.add(name, node.asBool());
    else if (node.isNull())
        options.add(name, "");
    else
        return false;
    return true;
}

} // unnamed namespace

Options PipelineReaderJSON::extractOptions(Json::Value& node)
{
    Options options;

    for (const std::string& name : node.getMemberNames())
    {
        Json::Value& subnode(node[name]);

        if (name == "plugin")
        {
            PluginManager<Stage>::loadPlugin(subnode.asString());

            // Don't actually put a "plugin" option on
            // any stage
            continue;
        }

        if (extractOption(options, name, subnode))
            continue;
        else if (subnode.isArray())
        {
            for (const Json::Value& val : subnode)
                if (!extractOption(options, name, val))
                    throw pdal_error("JSON pipeline: Invalid value type for "
                        "option list '" + name + "'.");
        }
        else if (subnode.isObject())
        {
            Json::FastWriter w;
            options.add(name, w.write(subnode));
        }
        else
            throw pdal_error("JSON pipeline: Value of stage option '" +
                name + "' cannot be converted.");
    }
    node.clear();
    return options;
}

} // namespace pdal
