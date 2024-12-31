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

#include <nlohmann/json.hpp>

#include <pdal/Filter.hpp>
#include <pdal/PipelineReaderJSON.hpp>
#include <pdal/PipelineManager.hpp>
#include <pdal/PluginManager.hpp>
#include <pdal/Options.hpp>
#include <pdal/util/FileUtils.hpp>
#include <pdal/util/Algorithm.hpp>
#include <pdal/util/Utils.hpp>

#include <memory>
#include <vector>

namespace pdal
{

PipelineReaderJSON::PipelineReaderJSON(PipelineManager& manager) :
    m_manager(manager)
{}


void PipelineReaderJSON::parsePipeline(NL::json& tree)
{
    TagMap tags;
    std::vector<Stage*> inputs;

    size_t last = tree.size() - 1;
    for (size_t i = 0; i < tree.size(); ++i)
    {
        NL::json& node = tree.at(i);

        FileSpec spec;
        std::string filename;
        std::string tag;
        std::string type;
        std::vector<Stage*> specifiedInputs;
        Options options;

        // strings are assumed to be filenames
        if (node.is_string())
        {
            filename = node.get<std::string>();
        }
        else
        {
            type = extractType(node);
            spec = extractFilename(node);
            //ABELL
            filename = spec.m_path;
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
            inputs.push_back(s);
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

    // Tell user if the pipeline seems wacky.
    const std::vector<Stage *> llist = m_manager.leaves();
    if (llist.size() > 1)
    {
        const LogPtr& log = m_manager.log();
        log->get(LogLevel::Error) << "Pipeline has multiple leaf nodes.\n";
        log->get(LogLevel::Error) << "Only the first of the following leaf nodes will be run.\n";
        for (Stage *s : llist)
        {
            std::string name = s->tag().size() ? s->tag() : s->getName();
            log->get(LogLevel::Error) << "    " << name << "\n";
        }
    }
}


void PipelineReaderJSON::readPipeline(std::istream& input)
{
    NL::json root;

    try
    {
        root = NL::json::parse(input, /* callback */ nullptr,
                                      /* allow exceptions */ true,
                                      /* ignore_comments */ true);
    }
    catch (NL::json::parse_error& err)
    {
        // Look for a right bracket -- this indicates the start of the
        // actual message from the parse error.
        std::string s(err.what());
        auto pos = s.find("]");
        if (pos != std::string::npos)
            s = s.substr(pos + 1);
        throw pdal_error("Pipeline:" + s);
    }

    auto it = root.find("pipeline");
    if (root.is_object() && it != root.end())
        parsePipeline(*it);
    else if (root.is_array())
        parsePipeline(root);
    else
        throw pdal_error("Pipeline: root element is not a pipeline.");
}


void PipelineReaderJSON::readPipeline(const std::string& filename)
{
    std::istream* input = Utils::openFile(filename);
    if (!input)
    {
        throw pdal_error("Pipeline: Unable to open stream for "
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
}


std::string PipelineReaderJSON::extractType(NL::json& node)
{
    std::string type;

    auto it = node.find("type");
    if (it != node.end())
    {
        NL::json& val = *it;
        if (!val.is_null())
        {
            if (val.is_string())
                type = val.get<std::string>();
            else
                throw pdal_error("JSON pipeline: 'type' must be specified as "
                    "a string.");
        }
        node.erase(it);
    }
    return type;
}


FileSpec PipelineReaderJSON::extractFilename(NL::json& node)
{
    FileSpec spec;

    auto it = node.find("filename");
    if (it == node.end())
        return spec;

    NL::json& val = *it;
    if (!val.is_null())
    {
        if (val.is_string())
            spec.m_path = val.get<std::string>();
        else if (val.is_object())
        {
            extractPath(val, spec);
            extractHeaders(val, spec);
            extractQuery(val, spec);
            if (!val.empty())
                throw pdal_error("JSON pipeline: Invalid item in filename object: " +
                    val.dump());
        }
        else
            throw pdal_error("JSON pipeline: 'filename' must be specified as a string.");
        node.erase(it);
    }
    return spec;
}

void PipelineReaderJSON::extractPath(NL::json& node, FileSpec& spec)
{
    auto it = node.find("path");
    if (it == node.end())
          throw pdal_error("JSON pipeline: 'filename' object must contain 'path' member.");

    NL::json& val = *it;
    if (!val.is_null())
    {
        if (val.is_string())
            spec.m_path = val.get<std::string>();
        else
            throw pdal_error("JSON pipeline: filename 'path' member must be specified "
                "as a string.");
        node.erase(it);
    }
}

void PipelineReaderJSON::extractHeaders(NL::json& node, FileSpec& spec)
{
    auto it = node.find("headers");
    if (it == node.end())
        return;

    NL::json& val = *it;
    if (!val.is_null())
    {
        spec.m_headers = extractStringList("headers", val);
        node.erase(it);
    }
}

void PipelineReaderJSON::extractQuery(NL::json& node, FileSpec& spec)
{
    auto it = node.find("query");
    if (it == node.end())
        return;

    NL::json& val = *it;
    if (!val.is_null())
    {
        spec.m_query = extractStringList("query", val);
        node.erase(it);
    }
}

StringList PipelineReaderJSON::extractStringList(const std::string& name, NL::json& node)
{
    StringList slist;

    auto error = [&name]()
    {
        throw pdal_error("JSON pipeline: '" + name + "' object must be a string or "
            "array of strings.");
    };

    if (node.is_string())
        slist.push_back(node.get<std::string>());
    else if (node.is_array())
    {
        for (size_t i = 0; i < node.size(); ++i)
        {
            NL::json& val = node.at(i);
            if (val.is_string())
                slist.push_back(val.get<std::string>());
            else
                error();
        }
    }
    else
        error();
    return slist;
}

std::string PipelineReaderJSON::extractTag(NL::json& node, TagMap& tags)
{
    std::string tag;

    auto it = node.find("tag");
    if (it != node.end())
    {
        NL::json& val = *it;
        if (!val.is_null())
        {
            if (val.is_string())
            {
                tag = val.get<std::string>();
                if (tags.find(tag) != tags.end())
                    throw pdal_error("JSON pipeline: duplicate tag '" +
                        tag + "'.");
            }
            else
                throw pdal_error("JSON pipeline: tag must be "
                    "specified as a string.");
        }
        node.erase(it);
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


std::vector<Stage *> PipelineReaderJSON::extractInputs(NL::json& node,
    TagMap& tags)
{
    std::vector<Stage *> inputs;
    std::string filename;

    auto it = node.find("inputs");
    if (it != node.end())
    {
        NL::json& val = *it;
        if (val.is_string())
            handleInputTag(val.get<std::string>(), tags, inputs);
        else if (val.is_array())
        {
            for (auto& input : val)
            {
                if (!input.is_string())
                    throw pdal_error("JSON pipeline: 'inputs' tag must "
                        " be specified as a string or array of strings.");
                handleInputTag(input.get<std::string>(), tags, inputs);
            }
        }
        else
            throw pdal_error("JSON pipeline: 'inputs' tag must "
                " be specified as a string or array of strings.");
        node.erase(it);
    }
    return inputs;
}

namespace
{

bool extractOption(Options& options, const std::string& name,
    const NL::json& node)
{
    if (node.is_string())
        options.add(name, node.get<std::string>());
    else if (node.is_number_unsigned())
        options.add(name, node.get<uint64_t>());
    else if (node.is_number_integer())
        options.add(name, node.get<int64_t>());
    else if (node.is_number_float())
        options.add(name, node.get<double>());
    else if (node.is_boolean())
        options.add(name, node.get<bool>());
    else if (node.is_array())
        options.add(name, node.get<NL::json::array_t>());
    else if (node.is_null())
        options.add(name, "");
    else
        return false;
    return true;
}

} // unnamed namespace

Options PipelineReaderJSON::extractOptions(NL::json& node)
{
    Options options;

    for (auto& it : node.items())
    {
        NL::json& subnode = it.value();
        const std::string& name = it.key();

        if (name == "plugin")
        {
            PluginManager<Stage>::loadPlugin(subnode.get<std::string>());

            // Don't actually put a "plugin" option on
            // any stage
            continue;
        }

        if (subnode.is_array())
        {
            for (const NL::json& val : subnode)
                if (val.is_object())
                    options.add(name, val);
                else if (!extractOption(options, name, val))
                    throw pdal_error("JSON pipeline: Invalid value type for "
                        "option list '" + name + "'.");
        }
        else if (subnode.is_object())
            options.add(name, subnode);
        else if (!extractOption(options, name, subnode))
            throw pdal_error("JSON pipeline: Value of stage option '" +
                name + "' cannot be converted.");
    }
    node.clear();
    return options;
}

} // namespace pdal
