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

#include "PipelineReaderXML.hpp"

#include <pdal/Filter.hpp>
#include <pdal/PipelineManager.hpp>
#include <pdal/PluginManager.hpp>
#include <pdal/Options.hpp>
#include <pdal/util/FileUtils.hpp>

#include <boost/property_tree/xml_parser.hpp>

#ifndef _WIN32
#include <wordexp.h>
#endif

namespace pdal
{

using namespace pdalboost::property_tree;

// ------------------------------------------------------------------------

// this class helps keep tracks of what child nodes we've seen, so we
// can keep all the error checking in one place
class PipelineReaderXML::StageParserContext
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
            throw pdal_error("PipelineReaderXML: expected Type element "
                "missing");
        if (m_numTypes > 1)
            throw pdal_error("PipelineReaderXML: extra Type element found");

        if (m_cardinality == None)
        {
            if (m_numStages != 0)
                throw pdal_error("PipelineReaderXML: found child stages where "
                    "none were expected");
        }
        if (m_cardinality == One)
        {
            if (m_numStages == 0)
                throw pdal_error("PipelineReaderXML: "
                    "expected child stage missing");
            if (m_numStages > 1)
                throw pdal_error("PipelineReaderXML: extra child stages found");
        }
        if (m_cardinality == Many)
        {
            if (m_numStages == 0)
                throw pdal_error("PipelineReaderXML: expected child stage "
                    "missing");
        }
    }

private:
    int m_numTypes;
    Cardinality m_cardinality; // num child stages allowed
    int m_numStages;
};


PipelineReaderXML::PipelineReaderXML(PipelineManager& manager) :
    m_manager(manager)
{}


Option PipelineReaderXML::parseElement_Option(const ptree& tree)
{
    // cur is an option element, such as this:
    //     <option>
    //       <name>myname</name>
    //       <description>my descr</description>
    //       <value>17</value>
    //     </option>
    // this function will process the element and return an Option from it

    map_t attrs;
    collect_attributes(attrs, tree);

    std::string name = attrs["name"];
    std::string value = tree.get_value<std::string>();
    Utils::trim(value);

    // filenames in the XML are fixed up as follows:
    //   - if absolute path, leave it alone
    //   - if relative path, make it absolute using the XML file's directory
    // The toAbsolutePath function does exactly that magic for us.
    if (name == "filename")
    {
        std::string path = value;
#ifndef _WIN32
        wordexp_t result;
        if (wordexp(path.c_str(), &result, 0) == 0)
        {
            if (result.we_wordc == 1)
                path = result.we_wordv[0];
        }
        wordfree(&result);
#endif
        if (!FileUtils::isAbsolutePath(path))
        {
            std::string abspath = FileUtils::toAbsolutePath(m_inputXmlFile);
            std::string absdir = FileUtils::getDirectory(abspath);
            path = FileUtils::toAbsolutePath(path, absdir);

            assert(FileUtils::isAbsolutePath(path));
        }
        return Option(name, path);
    }
    else if (name == "plugin")
    {
       PluginManager::loadPlugin(value);
    }
    return Option(name, value);
}


Stage *PipelineReaderXML::parseElement_anystage(const std::string& name,
    const ptree& subtree)
{
    if (name == "Filter")
    {
        return parseElement_Filter(subtree);
    }
    else if (name == "Reader")
    {
        return parseElement_Reader(subtree);
    }
    else if (name == "<xmlattr>")
    {
        // ignore: will parse later
    }
    else
    {
        throw pdal_error("PipelineReaderXML: encountered unknown stage type");
    }

    return NULL;
}


Stage *PipelineReaderXML::parseElement_Reader(const ptree& tree)
{
    Options options;
    StageParserContext context;
    std::string filename;
    context.setCardinality(StageParserContext::None);

    map_t attrs;
    collect_attributes(attrs, tree);

    auto iter = tree.begin();
    while (iter != tree.end())
    {
        const std::string& name = iter->first;
        const ptree& subtree = iter->second;

        if (name == "<xmlattr>")
        {
            // already parsed
        }
        else if (name == "Option")
        {
            Option option = parseElement_Option(subtree);
            if (option.getName() == "filename")
                filename = option.getValue();
            options.add(option);
        }
        else if (name == "Metadata")
        {
            // ignored for now
        }
        else
        {
            context.addUnknown(name);
        }
        ++iter;
    }

    std::string type;
    if (attrs.count("type"))
    {
        type = attrs["type"];
    }

    Stage& reader = m_manager.makeReader(filename, type);
    reader.removeOptions(options);
    reader.addOptions(options);

    context.addType();
    context.validate();
    return &reader;
}


Stage *PipelineReaderXML::parseElement_Filter(const ptree& tree)
{
    Options options;

    StageParserContext context;

    map_t attrs;
    collect_attributes(attrs, tree);

    std::vector<Stage*> prevStages;
    for (auto iter = tree.begin(); iter != tree.end(); ++iter)
    {
        const std::string& name = iter->first;
        const ptree& subtree = iter->second;

        if (name == "<xmlattr>")
        {
            // already parsed
        }
        else if (name == "Option")
        {
            Option option = parseElement_Option(subtree);
            options.add(option);
        }
        else if (name == "Metadata")
        {
            // ignored
        }
        else if (name == "Filter" || name == "Reader")
        {
            context.addStage();
            prevStages.push_back(parseElement_anystage(name, subtree));
        }
        else
        {
            context.addUnknown(name);
        }
    }

    std::string type;
    if (attrs.count("type"))
        type = attrs["type"];

    Stage& filter = m_manager.makeFilter(type);
    filter.removeOptions(options);
    filter.addOptions(options);
    for (auto sp : prevStages)
        filter.setInput(*sp);
    context.setCardinality(StageParserContext::Many);
    context.addType();
    context.validate();
    return &filter;
}


void PipelineReaderXML::parse_attributes(map_t& attrs, const ptree& tree)
{
    for (auto iter = tree.begin(); iter != tree.end(); ++iter)
    {
        std::string name = iter->first;
        std::string value = tree.get<std::string>(name);
        Utils::trim(value);

        attrs[name] = value;
    }
}


void PipelineReaderXML::collect_attributes(map_t& attrs, const ptree& tree)
{
    if (tree.count("<xmlattr>"))
    {
        const ptree& subtree = tree.get_child("<xmlattr>");
        parse_attributes(attrs, subtree);
    }
}


Stage *PipelineReaderXML::parseElement_Writer(const ptree& tree)
{
    Options options;
    StageParserContext context;
    std::string filename;

    map_t attrs;
    collect_attributes(attrs, tree);

    std::vector<Stage *> prevStages;
    for (auto iter = tree.begin(); iter != tree.end(); ++iter)
    {
        const std::string& name = iter->first;
        const ptree& subtree = iter->second;

        if (name == "<xmlattr>")
        {
            // already parsed -- ignore it
        }
        else if (name == "Option")
        {
            Option option = parseElement_Option(subtree);
            if (option.getName() == "filename")
                filename = option.getValue();
            options.add(option);
        }
        else if (name == "Metadata")
        {
            // ignored
        }
        else if (name == "Filter" || name == "Reader")
        {
            context.addStage();
            prevStages.push_back(parseElement_anystage(name, subtree));
        }
        else
        {
            context.addUnknown(name);
        }
    }

    std::string type;
    if (attrs.count("type"))
    {
        type = attrs["type"];
        context.addType();
    }

    context.validate();
    Stage& writer = m_manager.makeWriter(filename, type);
    for (auto sp : prevStages)
        writer.setInput(*sp);
    writer.removeOptions(options);
    writer.addOptions(options);
    return &writer;
}


void PipelineReaderXML::parseElement_Pipeline(const ptree& tree)
{
    Stage *stage = NULL;
    Stage *writer = NULL;

    map_t attrs;
    collect_attributes(attrs, tree);

    std::string version = "";
    if (attrs.count("version"))
        version = attrs["version"];
    if (version != "1.0")
        throw pdal_error("PipelineReaderXML: unsupported pipeline xml version");

    for (auto iter = tree.begin(); iter != tree.end(); ++iter)
    {
        const std::string& name = iter->first;
        const ptree subtree = iter->second;

        if (name == "Reader" || name == "Filter" )
        {
            stage = parseElement_anystage(name, subtree);
        }
        else if (name == "Writer")
        {
            writer = parseElement_Writer(subtree);
        }
        else if (name == "<xmlattr>")
        {
            // ignore it, already parsed
        }
        else
        {
            throw pdal_error("PipelineReaderXML: xml reader invalid child of "
                "ReaderPipeline element");
        }
    }

    if (writer && stage)
    {
        throw pdal_error("PipelineReaderXML: extra nodes at front of "
            "writer pipeline");
    }
}


void PipelineReaderXML::readPipeline(std::istream& input)
{
    ptree tree;

    xml_parser::read_xml(input, tree, xml_parser::no_comments);

    pdalboost::optional<ptree> opt(tree.get_child_optional("Pipeline"));
    if (!opt.is_initialized())
        throw pdal_error("PipelineReaderXML: root element is not Pipeline");
    parseElement_Pipeline(opt.get());
}


void PipelineReaderXML::readPipeline(const std::string& filename)
{
    m_inputXmlFile = filename;

    std::istream* input = FileUtils::openFile(filename);

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
            "  XML is invalid.";
        throw pdal_error(oss.str());
    }

    FileUtils::closeFile(input);

    m_inputXmlFile = "";
}


} // namespace pdal
